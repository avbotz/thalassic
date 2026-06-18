sudo ip link set dev end0 mtu 9000
sysctl -w net.core.rmem_max=10485760
sysctl -w net.core.wmem_max=10485760
