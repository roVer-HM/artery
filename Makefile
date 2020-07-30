PYTHON ?= python
PYTHON2 ?= python2
INET_DIR = ../inet4
SIMULTE_DIR = ../simulte
VANETZA_DIR = extern/vanetza
VEINS_DIR = ../veins

ifeq ($(MODE), debug)
CM_BUILD_TYPE=Debug
CM_BUILD_DIR=build/$(CM_BUILD_TYPE)
VANETZA_BUILD_DIR=$(VANETZA_DIR)/$(CM_BUILD_DIR)
else
CM_BUILD_TYPE=Release
CM_BUILD_DIR=build/$(CM_BUILD_TYPE)
VANETZA_BUILD_DIR=$(VANETZA_DIR)/$(CM_BUILD_DIR)
endif

N_CORES := $(shell nproc)

all: artery

allExtern: inet vanetza veins


cleanExtern:
	-$(MAKE) -C $(INET_DIR) cleanall
	-$(MAKE) -C $(VEINS_DIR) cleanall
	-rm -rf $(VANETZA_BUILD_DIR)

$(INET_DIR)/.oppfeaturestate: $(INET_DIR)/.oppfeatures
	cd $(INET_DIR); $(PYTHON) inet_featuretool repair

$(INET_DIR)/src/Makefile: $(INET_DIR)/.oppfeatures
	$(MAKE) -C $(INET_DIR) makefiles

inet: $(INET_DIR)/src/Makefile
	$(MAKE) -C $(INET_DIR)/src

$(SIMULTE_DIR)/src/Makefile: $(SIMULTE_DIR)/Version
	$(MAKE) -C $(SIMULTE_DIR) makefiles INET_PROJ=$(INET_DIR)
	$(MAKE) -C $(SIMULTE_DIR)/src depend

simulte: $(SIMULTE_DIR)/src/Makefile
	$(MAKE) -C $(SIMULTE_DIR)/src

$(VEINS_DIR)/src/Makefile: $(VEINS_DIR)/configure
	cd $(VEINS_DIR); $(PYTHON2) configure
	$(MAKE) -C $(VEINS_DIR)/src depend

veins: $(VEINS_DIR)/src/Makefile
	$(MAKE) -C $(VEINS_DIR)

$(VANETZA_BUILD_DIR):
	mkdir -p $(VANETZA_BUILD_DIR)

$(VANETZA_BUILD_DIR)/Makefile: $(VANETZA_BUILD_DIR)
	cd $<; cmake -DCMAKE_BUILD_TYPE=$(CM_BUILD_TYPE) -DBUILD_SHARED_LIBS=ON ../..

vanetza: $(VANETZA_BUILD_DIR)/Makefile
	$(MAKE) -j$(N_CORES) -C $(VANETZA_BUILD_DIR)

vanetza_clean: 
	rm -rf $(VANETZA_BUILD_DIR)

$(CM_BUILD_DIR): vanetza
	mkdir -p $(CM_BUILD_DIR)

$(CM_BUILD_DIR)/Makefile: $(CM_BUILD_DIR)
	cd $<; cmake -DCMAKE_BUILD_TYPE=$(CM_BUILD_TYPE) -DBUILD_SHARED_LIBS=ON ../..

artery: $(CM_BUILD_DIR)/Makefile
	$(MAKE) -j$(N_CORES) -C $(CM_BUILD_DIR)

artery_clean:
	rm -rf $(CM_BUILD_DIR)

# opp_makemake setup
MAKEMAKE=opp_makemake $(MMOPT)
INET_PROJ=../inet4
LTE_PROJ=../simulte
VEINS_PROJ=../veins
VEINS_INET_PROJ=../veins/subprojects/veins_inet
ARTERY_PROJ=.
VANETZA_PROJ=extern/vanetza



	

clean: vanetza_clean artery_clean 
	

cleanall: 
	rm -rf build
	rm -rf $(VANETZA_DIR)/build

makefiles:
		cd src && $(MAKEMAKE) --make-so -f --deep -o ARTERY -O out -pARTERY \
		 -KINET_PROJ=../$(INET_PROJ) \
		 -KLTE_PROJ=../$(LTE_PROJ) \
		 -KVEINS_PROJ=../$(VEINS_PROJ) \
		 -KVEINS_INET_PROJ=../$(VEINS_INET_PROJ) \
		 -KVANETZA_PROJ=../$(VANETZA_PROJ) \
		 -KARTERY_PROJ=../$(ARTERY_PROJ) \
		 -DINET_IMPORT -DVEINS_IMPORT -DVEINS_INET_IMPORT \
		 -I. \
		 -I$$\(ARTERY_PROJ\) \
		 -I$$\(ARTERY_PROJ\)/src/traci/sumo \
		 -I$$\(ARTERY_PROJ\)/extern/pybind11/include \
		 -I$$\(INET_PROJ\)/src \
		 -I$$\(LTE_PROJ\)/src \
		 -I$$\(VEINS_PROJ\)/src \
		 -I$$\(VEINS_PROJ\)/subprojects/veins_inet/src \
		 -I$$\(VEINS_PROJ\)/src/veins \
 		 -I$$\(VANETZA_PROJ\) \
 		 -I$$\(VANETZA_PROJ\)/vanetza/asn1/support \
 		 -I/usr/include/python3.6 \
		 -L$$\(INET_PROJ\)/src \
		 -L$$\(LTE_PROJ\)/src \
		 -L$$\(VEINS_PROJ\)/src \
		 -L$$\(VEINS_PROJ\)/subprojects/veins_inet/src \
		 -L$$\(VANETZA_PROJ\)/build/Debug/lib \
 		 -L$$\(VANETZA_PROJ\)/build/Release/lib \
		 -lINET$$\(D\) \
		 -llte$$\(D\) \
		 -lveins$$\(D\) \
		 -lveins_inet$$\(D\) \
		 -lvanetza_asn1$$\(D\) \
		 -Xartery/testbed \
		 -Xartery/transfusion

