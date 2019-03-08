REM objcopy -v -I binary -O ihex --change-addresses 0x12000000 ble_app_test_pca10056_s140.dat init_pkt.hex 
REM objcopy -v -I binary -O ihex --change-addresses 0x12001000 ble_app_test_pca10056_s140.bin   fw_bin.hex

nrfjprog -f nrf52 --qspicustominit --memrd 0x12000000 --w 8 --n 100

nrfjprog -f nrf52 --qspicustominit --memrd 0x12001000 --w 8 --n 100

