cmd_drivers/meep-pcie-ethernet/modules.order := {  :; } | awk '!x[$$0]++' - > drivers/meep-pcie-ethernet/modules.order
