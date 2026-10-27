struct meepeth_kthread {
	spinlock_t lock;
	char name[26];
	unsigned short cpu;
	unsigned short id;
	unsigned int kth_timeout;
	/**  flags for thread */
        unsigned long flag;
	/**  thread wait queue */
        wait_queue_head_t waitq;
	unsigned int schedule;
	struct task_struct *task;

	/**  device this thread takes care of */
        struct net_device * dev;

	/**  thread initialization handler */
        int (*finit)(struct meepeth_kthread *thp);

	/**  thread done handler */
        int (*fdone)(struct meepeth_kthread *thp);

};


/** lock thread macro */
#define lock_thread(thp)                spin_lock(&(thp)->lock)
/** un lock thread macro */
#define unlock_thread(thp)              spin_unlock(&(thp)->lock)
/** macro to wake up the qdma k thread */
#define meepeth_kthread_wakeup(thp) \
        do { \
                thp->schedule = 1; \
                wake_up_interruptible(&thp->waitq); \
        } while (0)

int meepeth_kthread_start(struct meepeth_kthread *thp, char *name, int id);
