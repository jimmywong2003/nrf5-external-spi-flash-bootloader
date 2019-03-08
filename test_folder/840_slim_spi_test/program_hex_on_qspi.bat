objcopy -v -I binary -O ihex --change-addresses 0x12000000 ble_app_test_pca10056_s140.dat init_pkt.hex 
objcopy -v -I binary -O ihex --change-addresses 0x12001000 ble_app_test_pca10056_s140.bin   fw_bin.hex

objcopy -v -I binary -O ihex --change-addresses 0x12020000 ble_app_test_pca10056_s140.dat init_pkt_2.hex 
objcopy -v -I binary -O ihex --change-addresses 0x12021000 ble_app_test_pca10056_s140.bin   fw_bin_2.hex

pause

nrfjprog -f nrf52 --qspicustominit --qspieraseall

pause

nrfjprog -f nrf52 --qspicustominit --program init_pkt.hex 
nrfjprog -f nrf52 --qspicustominit --program fw_bin.hex

pause