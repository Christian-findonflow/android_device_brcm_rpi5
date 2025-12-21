# CAN bus configuration for Seeed CAN-FD HAT v2
# Dual MCP2518FD controllers on SPI0

buses {
    name: "can0"
    native {
        ifname: "can0"
    }
    bitrate: 250000
}

buses {
    name: "can1"
    native {
        ifname: "can1"
    }
    bitrate: 250000
}
