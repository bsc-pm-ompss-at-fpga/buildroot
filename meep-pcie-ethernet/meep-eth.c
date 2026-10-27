#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/netpoll.h>
#include <linux/etherdevice.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/of_reserved_mem.h>
#include <net/pkt_sched.h>
#include <net/net_namespace.h>

#include "kthread.h"

//#define CONFIG_MEEPPCIETH_QEMU

#ifdef CONFIG_MEEPPCIETH_QEMU
    #define DEBUG
#endif

#define DEBUG_RX_QUEUE
// #define DEBUG_PACKETS


MODULE_DESCRIPTION("MEEP virtual Ethernet over PCIe");
MODULE_AUTHOR("MEEP 2022");
MODULE_LICENSE("GPL");

static long hw_buffers = (long) 0x60200000;
static long default_pckt_size = (long) 9000;
module_param(default_pckt_size, long, S_IRUGO);
MODULE_PARM_DESC(default_pckt_size, "Default data packet size, aka MTU size");

static long default_pckt_area = (long) 9216;    // @dcr I do not understand where this number comes from, it is used to allocate the skb buffer for rx packets
                                                // why so much overhead (216 bytes) ? Actually this is the size that it is allocated in shared memory 0x2400
module_param(default_pckt_area, long, S_IRUGO);
MODULE_PARM_DESC(default_pckt_size, "Default data packet area (9216 bytes)");

// With plain packets
//#define ONIC_SHMEM_SEND_OFFSET 0x1C000
//#define ONIC_SHMEM_RECV_OFFSET 0x1E000
//#define PACKET_SLOT_SIZE 2048

// With Jumbo packets
#define PACKET_SLOT_SIZE (0x2400)

// The LCM of packet size (0x2400) and PAGE_SIZE (0x1000) is 0x9000, which corresponds to 4 packets,
// so the number of packet slots should be multiple of 4 to make send and receive buffers be in separate pages

//#define NUM_PACKET_SLOTS    4
#define NUM_PACKET_SLOTS    (4*20)   // 80 packets per direction

#define ONIC_SHMEM_SEND_OFFSET 0x0
#define ONIC_SHMEM_RECV_OFFSET 0x100000 //1MB
#define ONIC_SHMEM_RECV_CNTRL_OFFSET 0x200000 //2MB

// @dcr This is the end of the buffer, it was previously defined by the hw_buffer_size variable, but it has to match the NUM_PACKET_SLOTS value
#define ONIC_SHMEM_SIZE		(ONIC_SHMEM_RECV_CNTRL_OFFSET + 4096)

#define PACKET_SHMEM_SEND_AREA_SIZE (PACKET_SLOT_SIZE*NUM_PACKET_SLOTS)
#define PACKET_SHMEM_RECV_AREA_SIZE (PACKET_SLOT_SIZE*NUM_PACKET_SLOTS)

#define ONIC_SHMEM_SEND_LIMIT (dp->as_base+PACKET_SHMEM_SEND_AREA_SIZE)
#define ONIC_SHMEM_RECV_LIMIT (dp->ar_base+PACKET_SHMEM_RECV_AREA_SIZE)

#define MEEP_MAGIC_STRING       "MEEP Ethernet-over-PCI"


/* queue support */

#define N_SKB (60)

struct sk_buff_q {
   struct sk_buff * skb [N_SKB];
   struct net_device * dev [N_SKB];
   int in;
   int out;
};

/**
 * Driver private memory
 */
struct meep_eth_dev_private
{
    int last_link;
    struct sk_buff * deferred_skb;
    spinlock_t dev_lock;

    volatile void __iomem * ar_base;
    volatile void __iomem * as_base;
    volatile void __iomem * ar;
    volatile void __iomem * as;

    volatile u64 __iomem * rx_cntrl;

    struct sk_buff_q * sk_buff_out;

    int queue_stopped;  // flag to indicate that the thread should try to wake up the queue
};

struct meepeth_kthread * meepeth_thread;

int insert_skb(struct sk_buff_q * q, struct sk_buff * s, struct net_device * d);
struct sk_buff * extract_skb(struct sk_buff_q * q);
int skb_available(struct sk_buff_q * q);
static void meep_eth_poll_controller(struct net_device *ndev);
static int meep_eth_do_xmit(struct sk_buff *skb, struct net_device *dev);
static int meep_eth_set_mac_address(struct net_device * dev, void * a);

struct sk_buff_q * new_skb_q(void)
{
    struct sk_buff_q * q;
    q = kzalloc(sizeof(struct sk_buff_q), GFP_KERNEL);
    if (q) 
    {
        q->in = 0;
        q->out = 0;
    }
    return q;
}

int insert_skb(struct sk_buff_q * q, struct sk_buff * s, struct net_device * d)
{
   int p = q->in;
   int n = (p+1) % N_SKB;
   if (n != q->out) {
      q->skb[p] = s;
      q->dev[p] = d;
      q->in = n;
      return (n > q->out)? n - q->out : N_SKB + n - q->out;
   }
   else
      return 0;
}

struct sk_buff * extract_skb(struct sk_buff_q * q)
{
   struct sk_buff * s;
   int p = q->out;
   if (p == q->in) return NULL;
   s = q->skb[p];
   q->out = (q->out + 1) % N_SKB;
#ifdef DEBUG
   pr_info ("dev 0x%lx  dev 0x%lx\n", (unsigned long) q->dev[p],
               (unsigned long) s->dev);
#endif
   return s;
}

int skb_available(struct sk_buff_q * q)
{
   return q->out != q->in;
}

/* end queue support */

static inline void xthread_reschedule(struct meepeth_kthread *thp)
{
    if (thp->kth_timeout) 
    {
#ifdef DEBUG_THREAD
	    printk(KERN_DEBUG "xthread: wait-timeout %d\n", thp->kth_timeout);
#endif
		wait_event_interruptible_timeout(thp->waitq, thp->schedule,
				msecs_to_jiffies(thp->kth_timeout));
	}
	else {
#ifdef DEBUG_THREAD
	    printk(KERN_DEBUG "xthread: wait\n");
#endif
		wait_event_interruptible(thp->waitq, thp->schedule);
	}
#ifdef DEBUG_THREAD
	printk(KERN_DEBUG "wake-up");
#endif
}

static inline int xthread_work_pending(struct meepeth_kthread * thp)
{
    return 0;
}




static int xthread_main(void *data)
{
	struct meepeth_kthread *thp = (struct meepeth_kthread *)data;

	pr_info("%s UP\n", thp->name);

	disallow_signal(SIGPIPE);

	if (thp->finit)
                thp->finit(thp);

	while (!kthread_should_stop()) 
        {
		// if we have more than 1 online cpus we devote 1 CPU to the driver
		int do_sleep = (num_online_cpus() == 1);

		/* any work to do? */
		lock_thread(thp);


		if (do_sleep)
		{	

			if (!xthread_work_pending(thp)) 
			{
				unlock_thread(thp);
#ifdef DEBUG_THREAD
				pr_info("%s, xthread_reschedule\n", thp->name);
#endif

				xthread_reschedule(thp);
				lock_thread(thp);
			}
			thp->schedule = 0;
#ifdef DEBUG_THREAD
			pr_info ("%s, do work\n", thp->name);
#endif

		}


		if (thp->dev != NULL) 
                {
/* @dcr transmission done in the xmit function
#ifdef DEBUG_THREAD
		    pr_info ("%s, meep_eth_xmit_controller\n", thp->name);
#endif
		    meep_eth_xmit_controller(thp->dev);
*/
                    struct meep_eth_dev_private *dp = netdev_priv(thp->dev);

                    if (dp->queue_stopped)
                        if (readw(dp->as) == 0)
                        {
                            // restart the queue if there is room for the next packet 
                            dp->queue_stopped = 0;
                            netif_start_queue(thp->dev);
                        }

#ifdef DEBUG_THREAD
		    pr_info ("%s, meep_eth_poll_controller\n", thp->name);
#endif
		    meep_eth_poll_controller(thp->dev);
#ifdef DEBUG_THREAD
		    pr_info ("%s, work done\n", thp->name);
#endif
		}

		unlock_thread(thp);


#ifdef DEBUG_THREAD
		pr_info("%s, scheduling\n", thp->name);
#endif

		schedule();

    }
    pr_info("%s, work done.\n", thp->name);
    if (thp->fdone)
        thp->fdone(thp);

    pr_info("%s, exit.\n", thp->name);
    return 0;
}


int meepeth_kthread_start(struct meepeth_kthread *thp, char *name, int id)
{
    int len;

    if (thp->task) 
    {
        pr_warn("kthread %s task already running?\n", thp->name);
        return -EINVAL;
    }

    len = snprintf(thp->name, sizeof(thp->name), "%s%d", name, id);
    if (len < 0)
        return -EINVAL;

    thp->id = id;

    spin_lock_init(&thp->lock);
    //INIT_LIST_HEAD(&thp->work_list);
    init_waitqueue_head(&thp->waitq);

    thp->task = kthread_create_on_node(xthread_main, (void *)thp,
                    cpu_to_node(thp->cpu), "%s", thp->name);

    if (IS_ERR(thp->task)) 
    {
        pr_err("kthread %s, create task failed: 0x%lx\n",
            thp->name, (unsigned long)IS_ERR(thp->task));
        thp->task = NULL;
        return -EFAULT;
    }

    kthread_bind(thp->task, thp->cpu);

    wake_up_process(thp->task);

    meepeth_thread = thp;

    return 0;
}


int meepeth_kthread_stop(struct meepeth_kthread *thp)
{
    int rv;

    if (!thp->task) 
    {
        pr_info("kthread %s, already stopped.\n", thp->name);
        return 0;
    }

    meepeth_kthread_wakeup(thp);

    rv = kthread_stop(thp->task);
    if (rv < 0) {
            pr_warn("kthread %s, stop err %d.\n", thp->name, rv);
            return rv;
    }

    pr_info("kthread %s, 0x%p, stopped.\n", thp->name, thp->task);
    thp->task = NULL;

    return 0;
}





// started from Xilinx Open NIC driver

#define XEL_RXBUFF_OFFSET       0x0000          /* Receive Buffer */

#define XEL_HEADER_OFFSET       12              /* Offset to length field */
#define XEL_HEADER_SHIFT        16              /* Shift value for length */

#define XEL_HW_HEADER_SIZE      14

/* General Ethernet Definitions */
#define XEL_ARP_PACKET_SIZE             28      /* Max ARP packet size */
#define XEL_HEADER_IP_LENGTH_OFFSET     16      /* IP Length Offset */

/* Transmit Packet Length Register (TPLR) */
#define XEL_TPLR_LENGTH_MASK    0x0000FFFF      /* Tx packet length */

/* Receive Packet Length Register (RPLR) */
#define XEL_RPLR_LENGTH_MASK    0x0000FFFF      /* Rx packet length */

// #define ALIGNMENT               8

/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
// #define BUFFER_ALIGN(adr) ((ALIGNMENT - ((u64)adr)) % ALIGNMENT)


#define TX_Q_LIMIT 1

#define ETHSAMPLE_FEATURES 0 // (NETIF_F_SG)

/* @dcr this looks unnecessary
  
 struct meep_eth_q_private {
        struct net_device       *dev;
        unsigned long long area[11];
} ____cacheline_aligned_in_smp;
*/




static int meep_eth_open(struct net_device *dev);
static int meep_eth_close(struct net_device *dev);
static void meep_eth_get_stats64(struct net_device *dev,
                                  struct rtnl_link_stats64 *storage);

static int meep_eth_xmit(struct sk_buff *skb, struct net_device *dev);

static int meep_eth_validate_addr(struct net_device * dev);
static int meep_eth_get_vf_config(struct net_device *dev,
                              int vf, struct ifla_vf_info *ivi);
static int meep_eth_set_vf_link_state(struct net_device *dev,
                              int vf, int link_state);
static int meep_eth_dev_init(struct net_device *dev);
static int meep_eth_change_mtu(struct net_device *dev, int mtu);

static void consume_rx_packet(struct meep_eth_dev_private * dp);
static void* peek_rx_packet_slot(struct meep_eth_dev_private * dp, int slot);
static void* peek_tx_packet_slot(struct meep_eth_dev_private * dp, int slot);

/**
 * Gets the pointer to the rx packet slot
 */
static void* peek_rx_packet_slot(struct meep_eth_dev_private * dp, int slot)
{
    unsigned char* p = (unsigned char*) dp->ar_base;
    short __iomem * psize = (short __iomem *) &p[PACKET_SLOT_SIZE*slot];
    if (*psize == 0)
        return NULL;
    return psize;
}


/**
 * Advances the pointer until we find an incoming packet.
 * This function is called after we are sure that there is an incoming packet
 * (after calling to any_rx_packet_available) even if it is not located in 
 * the head of the circular queue.
 * This can happen is the synchronization is lost between the two endpoints.
 */
static void* get_rx_packet(struct meep_eth_dev_private * dp)
{
    int i = 0;
    while (*((short __iomem*)dp->ar) == 0)
    {
        consume_rx_packet(dp);
        i++;

        if (i > NUM_PACKET_SLOTS)
        {
            // All slots seem empty but previous call (to any_rx_packet_available)
            // informed that there was a packet
            printk(KERN_DEBUG "we checked all possible rx slots. This is inconsistent");
        }
    }

    return dp->ar;
}

/**
 * consume the incoming rx packet. This means that the slot will be free (for  
 *  future use) and the read pointer is advanced to the next slot 
 */
static void consume_rx_packet(struct meep_eth_dev_private * dp)
{
	writew(0, dp->ar); 
    
    dp->ar += PACKET_SLOT_SIZE;
    if ((uintptr_t) dp->ar >= (uintptr_t) (ONIC_SHMEM_RECV_LIMIT))
        dp->ar = dp->ar_base;
}

/**
 * Discard all incoming packets
 */
static void flush_rx_packets(struct meep_eth_dev_private * dp)
{
    int i;
    for (i=0; i < NUM_PACKET_SLOTS; i++)
    {
        short __iomem* address = (short*)((uintptr_t)dp->ar_base + i*PACKET_SLOT_SIZE);
        *address = 0;
    }

    dp->ar = dp->ar_base;
}

/**
 * Gets the pointer to the rx packet slot
 */
static void* peek_tx_packet_slot(struct meep_eth_dev_private * dp, int slot)
{
    unsigned char* p = (unsigned char*) dp->as_base;
    short __iomem * psize = (short __iomem *) &p[PACKET_SLOT_SIZE*slot];
    if (*psize == 0)
        return NULL;
    return psize;
}


/**
 * 
 */
static void* get_free_tx_packet_slot(struct meep_eth_dev_private * dp)
{
    int i = 0;
    while (*((short __iomem*)dp->as) != 0)
    {
        dp->as += PACKET_SLOT_SIZE;

        if ((unsigned long) dp->as >= (unsigned long) (ONIC_SHMEM_SEND_LIMIT) )
            dp->as = dp->as_base;

        i++;

        if (i > NUM_PACKET_SLOTS)
        {
            // All slots seem empty but previous call (to any_rx_packet_available)
            // informed that there was a packet
            printk(KERN_DEBUG "we checked all possible tx slots. This is inconsistent");
        }
    }

    return dp->as;
}

/**
 * Enqueue a tx packet
 */
static void enqueue_tx_packet(struct meep_eth_dev_private * dp, short sz)
{
    short __iomem* address = dp->as;
    *address = sz; //skb->len;

    dp->as += PACKET_SLOT_SIZE;

    if ((unsigned long) dp->as >= (unsigned long) (ONIC_SHMEM_SEND_LIMIT) )
        dp->as = dp->as_base;
}

/**
 * Discard all incoming packets
 */
static void flush_tx_packets(struct meep_eth_dev_private * dp)
{
    int i;
    for (i=0; i < NUM_PACKET_SLOTS; i++)
    {
        short __iomem* address = (short __iomem*)((uintptr_t)dp->as_base + i*PACKET_SLOT_SIZE);
        *address = 0;
    }

    dp->as = dp->as_base;
}

static const struct net_device_ops meep_eth_ops = {
    .ndo_open               = meep_eth_open,
    .ndo_stop               = meep_eth_close,
    .ndo_change_mtu	    = meep_eth_change_mtu,
    .ndo_get_stats64        = meep_eth_get_stats64,
    .ndo_start_xmit         = meep_eth_xmit,        // transmission
    .ndo_validate_addr      = meep_eth_validate_addr,
    .ndo_get_vf_config      = meep_eth_get_vf_config,
    .ndo_set_vf_link_state  = meep_eth_set_vf_link_state,
    .ndo_init               = meep_eth_dev_init,
    .ndo_set_mac_address    = meep_eth_set_mac_address,
    //.ndo_poll_controller    = meep_eth_poll_controller, // reception
};

int meep_eth_change_mtu(struct net_device *dev, int mtu)
{
    netdev_info(dev, "Requested MTU = %d\n", mtu);
    dev->mtu = mtu;
    return 0;
}

static int meep_eth_dev_init(struct net_device *dev)
{
    struct meep_eth_dev_private * dp = netdev_priv(dev);
    // struct meep_eth_q_private * txp;

    printk(KERN_DEBUG "meep_eth_dev_init starts\n");
    spin_lock_init(&dp->dev_lock);

   // txp = kcalloc(dev->num_tx_queues, sizeof(*txp), GFP_KERNEL);
   /*
   if (txp != NULL)
   {
      // dp->d_priv = txp;
      dp->last_link = 0;
      dp->deferred_skb = NULL;
      spin_lock_init(&dp->dev_lock);
      netpoll_poll_enable(dev);
      netif_start_queue(dev);
      netif_trans_update(dev); // prevent tx timeout 
      netif_wake_queue(dev);
      netif_carrier_on(dev);
   }
   else
   {
      printk(KERN_DEBUG "meep_eth_dev_init ends (ENOMEM)\n");
      return -ENOMEM;
   }*/

    printk(KERN_DEBUG "meep_eth_dev_init ends (ok)\n");
    return 0;
}


static int meep_eth_open(struct net_device *dev)
{
    struct meep_eth_dev_private * dp = netdev_priv(dev);
    writeq(1, dp->rx_cntrl); //Enable packet receive
    //flush_tx_packets(dp);
    //flush_rx_packets(dp);

    printk(KERN_DEBUG "meep_eth_open ok\n");
    return 0;
}

static int meep_eth_close(struct net_device *dev)
{
	struct meep_eth_dev_private * dp = netdev_priv(dev);
	writeq(0, dp->rx_cntrl);
    netpoll_poll_disable(dev);

    printk(KERN_DEBUG "meep_eth_close ok\n");
    return 0;
}

static int meep_eth_set_vf_link_state(struct net_device *dev, int vf, int link_state)
{
   struct meep_eth_dev_private *dp = netdev_priv(dev);

   printk(KERN_DEBUG "meep_eth_set_vf_link_state (vf %d, link_state %d)\n",
           vf, link_state);

   if (dp->last_link != link_state) 
   {
      printk (KERN_DEBUG "netdev new state (%d) -> %d\n",
                 dp->last_link, link_state);
      dp->last_link = link_state;
   }
   else {
      printk (KERN_DEBUG "netdev new state %d -> %d\n",
                 dp->last_link, link_state);
   }

   return 0;
}


void display_packet(struct sk_buff * skb, int size)
{
        char str[160];
        unsigned char * d = skb->head;
        int i;

        for (i=0; i < ((size > 32) ? 32 : size); i += 16) {
           sprintf(str, "%lx: %02x %02x %02x %02x %02x %02x %02x %02x "
                        "%02x %02x %02x %02x %02x %02x %02x %02x",
                (unsigned long) &d[i], d[i], d[i+1], d[i+2], d[i+3],
                d[i+4], d[i+5], d[i+6], d[i+7],
                d[i+8], d[i+9], d[i+10], d[i+11],
                d[i+12], d[i+13], d[i+14], d[i+15]);
           printk(KERN_DEBUG "%s\n", str);
        }
	printk (KERN_DEBUG "... packet size %d\n", size);
}

void display_packet_details(struct sk_buff * skb, int size)
{
        unsigned char * d = skb->data;

	printk(KERN_DEBUG "%lx:\n", (unsigned long) &d[0]);
        printk(KERN_DEBUG "%x:%x:%x:%x:%x:%x\n",
                        d[0], d[1], d[2], d[3], d[4], d[5]);
        printk(KERN_DEBUG "%x:%x:%x:%x:%x:%x\n",
                        d[6], d[7], d[8], d[9], d[10], d[11]);
        printk(KERN_DEBUG "%x %x\n", d[12], d[13]);
        printk(KERN_DEBUG "%x %x\n", d[14], d[15]);
}



#ifdef DEBUG
static unsigned char te(unsigned char c)
{
        unsigned char e = ((c>=32) && (c<127))? c : '?';
        return e;
}
#endif


/**
 * Sends a net buffer to the tx queue
 *
 * @param skb buffer to send
 * @param dev 
 * @param dp private area
 * @param sz size of the packet (payload)
 *
 * @todo it does not make much sense to pass the a pointer (it is already 
 * a global variable of the driver) and the sz 
 */
static netdev_tx_t onic_send_frame(struct sk_buff *skb,
		struct net_device *dev, 
                struct meep_eth_dev_private* dp,
                int sz)
{
	int i;
	netdev_tx_t rv;

	short __iomem * address = get_free_tx_packet_slot(dp);
	uint64_t __iomem * ioa = (uint64_t __iomem*)address;
	//char * ad = a;

	if (*address == 0) 
        {
            // size of the packet should be zero

            // empty slot
            
		for (i = 0; i < sz/sizeof(uint64_t); ++i) {
			uint64_t w = skb->data[i*8];
			w |= (uint64_t)skb->data[i*8 + 1] << 8*1;
			w |= (uint64_t)skb->data[i*8 + 2] << 8*2;
            w |= (uint64_t)skb->data[i*8 + 3] << 8*3;
            w |= (uint64_t)skb->data[i*8 + 4] << 8*4;
            w |= (uint64_t)skb->data[i*8 + 5] << 8*5;
            w |= (uint64_t)skb->data[i*8 + 6] << 8*6;
            w |= (uint64_t)skb->data[i*8 + 7] << 8*7;
			ioa[i+8] = w;
        }
		if (sz%sizeof(uint64_t) != 0) memcpy(ioa + 8 + sz/sizeof(uint64_t), (uint64_t*)skb->data + sz/sizeof(uint64_t), sz%sizeof(uint64_t));
		
		enqueue_tx_packet(dp, sz < 64 ? 64 : sz);
		rv = NETDEV_TX_OK;
	}
	else 
        {
            // This will never happen
            pr_info ("ERROR!!! onic_send_frame NETDEV_TX_BUSY\n");
            rv = NETDEV_TX_BUSY;
	}

	return rv;
}

/**
 * Entry point to transmits a packet.
 */
static int meep_eth_xmit(struct sk_buff *skb, struct net_device *dev)
{
    //int sz;
    unsigned long flags;
    struct meep_eth_dev_private * dp = netdev_priv(dev);

    // we stop the sending queue by default
    netif_stop_queue(dev);

    spin_lock_irqsave(&dp->dev_lock, flags);
   
    meep_eth_do_xmit(skb, dev);

    /* To exclude tx timeout */
    // netif_trans_update(dev); // prevent tx timeout

    spin_unlock_irqrestore(&dp->dev_lock, flags);
    
    if (readw(dp->as) == 0)
        // we only restart the queue if there is room for the next packet 
        netif_start_queue(dev);
    else
        // indicate that the thread should try to wake up the queue
        dp->queue_stopped = 1;

    return NETDEV_TX_OK;
}

/**
 * Do the transmission of the skb buffer.
 * When we call this function we are sure that there is a free tx slot available
 * @param skb buffer to transmit
 * @param dev ?
 * @return ?
 */
static int meep_eth_do_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct meep_eth_dev_private *dp = netdev_priv(dev);
   netdev_tx_t rv;
   unsigned int len;
   ssize_t sz;

   len = skb->len;

   // put the packet available for the network
   sz = (skb->len < 10000)? skb->len : 10000;

   //sz -= 14;

   /* Send to the network device */
#ifdef DEBUG
   printk (KERN_DEBUG "Sending to network (size %ld -> %ld)\n", sz, sz);
#endif

   rv = onic_send_frame(skb, skb->dev, dp, sz);

   /********************************/

   //send_data_available = sz;
   skb_tx_timestamp(skb);
   dev_consume_skb_any(skb);   // check...

   if (rv == NETDEV_TX_OK) 
    {
	dev->stats.tx_packets++;
   	dev->stats.tx_bytes += sz; //len;
        
   }
   else 
    {
      dev->stats.tx_dropped++;
   }


   return rv; //NETDEV_TX_OK;
}


/**
 * Receive a packet from the interchange buffer
 *
 * @param dp driver private pointer
 * @param data pointer to the receive buffer
 * @param maxlen maximum length to receive
 */
static int meep_eth_recv_data(struct meep_eth_dev_private *dp,
                                u8 * data, u16 size)
{
   volatile u64 __iomem * addr = dp->ar;
	int i;
#ifdef DEBUG
	        printk(KERN_DEBUG "   reading from system %d bytes\n", length);
#endif

	for (i = 0; i < size/sizeof(u64); ++i) {
		u64 d = addr[8+i];
		data[i*8 + 0] = d >> 8*0;
		data[i*8 + 1] = d >> 8*1;
		data[i*8 + 2] = d >> 8*2;
        data[i*8 + 3] = d >> 8*3;
        data[i*8 + 4] = d >> 8*4;
        data[i*8 + 5] = d >> 8*5;
        data[i*8 + 6] = d >> 8*6;
        data[i*8 + 7] = d >> 8*7;
	}
	if (size%sizeof(u64) != 0) memcpy((u64*)data + size/sizeof(u64), addr+8+size/sizeof(u64), size%sizeof(u64));

	/* Acknowledge the frame */
#ifdef DEBUG
	pr_info("ack_packet_available: base %lx curr %lx\n", (unsigned long) dp->ar_base, (unsigned long) addr);
#endif
	consume_rx_packet(dp);

	return size;
}

/**
 * This function allocates memory for a socket buffer, fills it with data
 * received and hands it over to the TCP/IP stack.
 *
 * @return the number of packets received
 */
static int meep_eth_rx_handler(struct net_device *dev)
{
    int processed = 0;
    struct meep_eth_dev_private *dp = netdev_priv(dev);
    struct sk_buff *skb;
    u16 size;
    int res;

    size = readw(dp->ar);

    if (size != 0) 
    {
        // there is an incoming packet in the queue

#ifdef DEBUG_THREAD
	pr_info("c alloc_skb %d\n", size);
#endif

        skb = netdev_alloc_skb_ip_align(dev, default_pckt_area);

        if (!skb) 
        {
            /* Couldn't get memory. */
            dev->stats.rx_dropped++;
            printk(KERN_DEBUG "Could not allocate receive buffer\n");
            return processed;
        }

        //skb_set_network_header(skb, 0);
        /* A new skb should have the data halfword aligned, but this code is
         * here just in case that isn't true. Calculate how many
         * bytes we should reserve to get the data to start on a word
         * boundary
         */
        /*align = (unsigned int) BUFFER_ALIGN(skb->data);
        if (align)
            skb_reserve(skb, align);*/
		// this will copy the whole ethernet packet
		meep_eth_recv_data(dp, (u8*) skb->data, size);

#ifdef DEBUG
        printk(KERN_DEBUG "\nRECEIVING packet (alloc size %d bytes, len %d)\n", size, len);
#endif

        skb_put(skb, size);      /* Tell the skb how much data we got */
        skb->protocol = eth_type_trans(skb, dev);

        //skb->ip_summed = CHECKSUM_NONE;
        if (skb->ip_summed != CHECKSUM_NONE)
            printk(KERN_DEBUG "would bug on checksum_none (%d)\n", skb->ip_summed);

        dev->stats.rx_packets++;
        dev->stats.rx_bytes += size;

        if (!skb_defer_rx_timestamp(skb)) 
        {
                netif_trans_update(dev); /* prevent tx timeout */
                //netif_wake_queue(dev);
#ifdef DEBUG
                printk(KERN_DEBUG 
                        "   send the packet upstream len %d  skb->len %d\n", 
                        len, skb->len);
#endif
		res = netif_rx(skb);
                //res = netif_receive_skb_core(skb);
		if (res != NET_RX_SUCCESS)
                   printk(KERN_DEBUG "   netif_receive_skb %d (%d %d) drop\n",
				   res, NET_RX_SUCCESS, NET_RX_DROP);
#ifdef DEBUG
		else
		   printk(KERN_DEBUG "   netif_receive_skb ok\n");
#endif
		processed++;
        }

	else
                printk(KERN_DEBUG "   deferring the packet\n");
    }
    return processed;
}



static irqreturn_t meep_eth_interrupt(int irq, struct net_device *ndev)
{
#ifdef DEBUG_RECV
   printk(KERN_DEBUG "meep_eth_interrupt %d\n", irq);
#endif

   return meep_eth_rx_handler(ndev);
   //return IRQ_HANDLED;
}

static void meep_eth_poll_controller(struct net_device *ndev)
{
   int count = 0;
#ifdef DEBUG_THREAD
   printk (KERN_DEBUG "meep_eth_poll_controller managing interrupts (%d)\n", 
		   ndev->irq);
#endif
   //disable_irq(ndev->irq);
   while ((meep_eth_interrupt(ndev->irq, ndev) > 0) && (count < 2)) count++;
   //enable_irq(ndev->irq);
#ifdef DEBUG_THREAD
   printk (KERN_DEBUG "end of meep_eth_poll_controller\n");
#endif
}


static void meep_eth_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
   //printk(KERN_DEBUG "meep_eth_get_stats64 storage %p\n", storage);

   memcpy(storage, &dev->stats, sizeof(struct rtnl_link_stats64));

   return ;
}

static int meep_eth_validate_addr(struct net_device * dev)
{
   int res;
   printk(KERN_DEBUG "meep_eth_validate_addr called\n");
   res = eth_validate_addr(dev);
   printk(KERN_DEBUG "meep_eth_validate_addr returns %d\n", res);
   return res;
}

static int meep_eth_get_vf_config(struct net_device *dev,
                              int vf, struct ifla_vf_info *ivi)
{
   printk(KERN_DEBUG "meep_eth_get_vf_config (vf %d)\n", vf);

   return 0;
}

static void meep_eth_dev_free(struct net_device *dev)
{
        //struct meep_eth_dev_private *dp = netdev_priv(dev);
        printk(KERN_DEBUG " meep_eth_dev_free starts\n");

        //kfree(dp->d_priv);
        printk(KERN_DEBUG "meep_eth_dev_free ends\n");
}

static int meep_eth_set_mac_address(struct net_device * dev, void  * p)
{
	return eth_mac_addr(dev, p);
}

static void meep_eth_setup(struct net_device *dev)
{
	struct meep_eth_dev_private *dp = netdev_priv(dev);

	int i;
	volatile void __iomem * ioaddr;
	volatile void __iomem * phys_mem;
	volatile short * __iomem ioa;
	u64 mac_addr;
	struct sockaddr addr;
	struct device_node onic_node;
	struct reserved_mem* onic_mem;
        // unsigned char addr[8];
        printk(KERN_DEBUG "meep_eth_setup\n");
        /* Initialize the device structure. */
        dev->netdev_ops = &meep_eth_ops;

	/* Fill in device structure with ethernet-generic values. */
        ether_setup(dev);
        dev->tx_queue_len = TX_Q_LIMIT;

        dev->features |= ETHSAMPLE_FEATURES;
        dev->hw_features |= dev->features;
        dev->hw_enc_features |= dev->features;
        dev->vlan_features |= ETHSAMPLE_FEATURES & ~(NETIF_F_HW_VLAN_CTAG_TX |
                                                      NETIF_F_HW_VLAN_STAG_TX);

	dev->type = ARPHRD_ETHER;
        dev->flags |= IFF_BROADCAST;
        dev->flags |= IFF_MULTICAST;
        dev->priv_flags &= ~IFF_TX_SKB_SHARING;
        netif_keep_dst(dev);
        //eth_hw_addr_random(dev);
	
	dev->needs_free_netdev = true;
        dev->priv_destructor = meep_eth_dev_free;

        dev->min_mtu = 60;
	dev->mtu = default_pckt_size; 
        dev->max_mtu = 9000; /// 1548;

#ifndef CONFIG_MEEPPCIETH_QEMU
    	onic_node.full_name = "onic_pool_node";
		onic_mem = of_reserved_mem_lookup(&onic_node);
    	if (onic_mem) 
	{
      		pr_info("assigned reserved memory node %s at 0x%llx, size 0x%llx\n", onic_mem->name, onic_mem->base, onic_mem->size);
      		hw_buffers = onic_mem->base;
    	}
    	else 
		pr_err("reserved memory node %s is not found in DT\n", onic_node.full_name);

	if (hw_buffers != 0) 
	{
		// Map the buffers that we will use to communicate with the Host System
		ioaddr = ioremap(hw_buffers, ONIC_SHMEM_SIZE); 
		phys_mem = (void* __iomem)hw_buffers;
	}
	else 
	{
		printk(KERN_ERR "WARNING! behaviour is unpredictable with this type of memory\n");
		ioaddr = dma_alloc_coherent(dev->dev.parent,
						ONIC_SHMEM_SIZE,
						(dma_addr_t*) &phys_mem, GFP_KERNEL);
	}
	pr_info ("ioaddr 0x%lx   phys_mem 0x%lx\n",
                    (unsigned long) ioaddr, (unsigned long) phys_mem);
	
	if (IS_ERR(ioaddr))
	       return;

	ioa = (short * __iomem) ioaddr;
#endif

#ifndef CONFIG_MEEPPCIETH_QEMU
	// In QEMU we use the private device memory to emulate the queues
	// but in the real device we use the physical address range
	dp->as_base = ioaddr + ONIC_SHMEM_SEND_OFFSET;
	dp->ar_base = ioaddr + ONIC_SHMEM_RECV_OFFSET;

	pr_info("meep: start buffer: %016lX \n", hw_buffers  );
	pr_info("meep: Tx buffer: %016lX \n", hw_buffers + ONIC_SHMEM_SEND_OFFSET );
	pr_info("meep: Rx buffer: %016lX \n", hw_buffers + ONIC_SHMEM_RECV_OFFSET );
	pr_info("meep: end buffer: %016lX \n", hw_buffers + ONIC_SHMEM_SIZE);
	pr_info("meep: num slots: %d \n", NUM_PACKET_SLOTS);
    pr_info("meep: cleaning queues\n");
    for (i = 0; i < NUM_PACKET_SLOTS; ++i) {
        ioa[(ONIC_SHMEM_SEND_OFFSET + i*PACKET_SLOT_SIZE)/sizeof(short)] = 0;
    }
    for (i = 0; i < NUM_PACKET_SLOTS; ++i) {
        ioa[(ONIC_SHMEM_RECV_OFFSET + i*PACKET_SLOT_SIZE)/sizeof(short)] = 0;
    }
    pr_info("meep: done\n");
#else
	printk(KERN_DEBUG "meep: allocated buffers in private memory for QEMU tx: %pS rx: %pS\n", dp->as_base, dp->ar_base); 
#endif
	dp->as = dp->as_base;
	dp->ar = dp->ar_base;
    dp->rx_cntrl = (u64*)ioaddr + ONIC_SHMEM_RECV_CNTRL_OFFSET/sizeof(u64);

	addr.sa_family = AF_UNSPEC;

	//mac_addr is stored in big endian because network packets are
	mac_addr = readq(dp->rx_cntrl);
	pr_info("Got mac addr 0x%llX\n", mac_addr);

	addr.sa_data[0] = (mac_addr >> 8*0) & 0xFF;
	addr.sa_data[1] = (mac_addr >> 8*1) & 0xFF;
	addr.sa_data[2] = (mac_addr >> 8*2) & 0xFF;
	addr.sa_data[3] = (mac_addr >> 8*3) & 0xFF;
	addr.sa_data[4] = (mac_addr >> 8*4) & 0xFF;
	addr.sa_data[5] = (mac_addr >> 8*5) & 0xFF;
	meep_eth_set_mac_address(dev, (void*) &addr);

	printk(KERN_DEBUG "meep_eth_setup ends\n");
}

static int meep_eth_validate(struct nlattr *tb[], struct nlattr *data[],
                        struct netlink_ext_ack *extack)
{
    int res = 0;
    printk(KERN_DEBUG "meep_eth_validate starts\n");
    if (tb[IFLA_ADDRESS]) 
    {
            if (nla_len(tb[IFLA_ADDRESS]) != ETH_ALEN)
                    res = -EINVAL;
            else if (!is_valid_ether_addr(nla_data(tb[IFLA_ADDRESS])))
                    res = -EADDRNOTAVAIL;
    }

    printk(KERN_DEBUG "meep_eth_valiadate ends (error %d)\n", res);
    return res;
}


static struct rtnl_link_ops meep_eth_device __read_mostly = 
{
        .kind           = "meepo_eth",
        .priv_size      = sizeof(struct meep_eth_dev_private),
        .setup          = meep_eth_setup,
        .validate       = meep_eth_validate,
};


/* Init / fini functions */

static int __init meep_eth_init_one(int id)
{
	struct meepeth_kthread * th;
        struct net_device *dev_meep_eth;
        int err;
	struct meep_eth_dev_private *dp;
	//int len;
	//
	struct sk_buff_q * sk_buff_out = new_skb_q();
	if (sk_buff_out == NULL) {
		err = -ENOMEM;
		goto err4;
	}

	th = kzalloc ( 1 * sizeof (struct meepeth_kthread), GFP_KERNEL);
	if (th == NULL) {
		err = -ENOMEM;
		goto err3;
	}

	th->cpu = 0;
	th->kth_timeout = 15;

	err = meepeth_kthread_start(th, "meep_eth_th", 0);
	pr_info ("meepeth_kthread_start returns %d\n", err);

	if (err < 0) goto err2;

        dev_meep_eth = alloc_netdev(sizeof(struct meep_eth_dev_private),
                         "meepeth%d", NET_NAME_UNKNOWN, meep_eth_setup);

        if (!dev_meep_eth) {
	   err = -ENOMEM;
	   goto err2;
	}


        dev_meep_eth->rtnl_link_ops = &meep_eth_device;
        err = register_netdevice(dev_meep_eth);
        if (err < 0)
             goto err;

	th->dev = dev_meep_eth;

        // @dcr assign the buffer pointer to private area
        dp = netdev_priv(dev_meep_eth);
        dp->sk_buff_out = sk_buff_out;
        dp->queue_stopped = 0;

        return 0;
err:
        free_netdev(dev_meep_eth);
err2:
	kfree(th);
err3:
	kfree(sk_buff_out);
err4:

        return err;
}

static int __init meep_eth_init(void) 
{
  int res = 0;
  printk(KERN_DEBUG "meep_eth_init\n");

#ifdef CONFIG_MEEPPCIETH_QEMU
  printk(KERN_DEBUG "----------------------------------\n");
  printk(KERN_DEBUG "| Special EMULATION MODE on QEMU |\n");
  printk(KERN_DEBUG "----------------------------------\n");
#endif

  down_write(&pernet_ops_rwsem);
  rtnl_lock();
    res = __rtnl_link_register(&meep_eth_device);
    if (res >= 0) {
      // correct
      res = meep_eth_init_one(0);
    }

    rtnl_unlock();
    up_write(&pernet_ops_rwsem);
  printk (KERN_DEBUG "init returns %d\n", res);
  return res;
}

static void __exit meep_eth_exit(void)
{
  printk(KERN_DEBUG "Bye, ...\n");

   meepeth_kthread_stop(meepeth_thread);
   kfree(meepeth_thread);
   meepeth_thread = NULL;

    rtnl_link_unregister(&meep_eth_device);

    printk(KERN_DEBUG "Bye, cruel world\n");
}

module_init(meep_eth_init);
module_exit(meep_eth_exit);

