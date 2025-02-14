INC_DIR = /usr/local/include/BHand
LIB_DIR = /usr/local/lib

install:
	mkdir -p $(INC_DIR)
	cp include/BHand/BHand.h $(INC_DIR)
	cp include/BHand/BHandDef.h $(INC_DIR)
	cp lib/libBHand.so $(LIB_DIR)

uninstall:
	rm -f $(INC_DIR)/BHand.h
	rm -f $(INC_DIR)/BHandDef.h
	rm -f $(LIB_DIR)/libBHand.so

