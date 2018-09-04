all:
	@make -C lcmtypes                                                          
	@make -C balancebot --no-print-directory
	@make -C test_motors --no-print-directory
	@make -C test_lcm --no-print-directory
	@make -C optitrack/common                                                  
	@make -C optitrack                                                         

lcmspy:	
	/bin/bash setenv.sh
	@make -C lcmtypes                                                          
	@make -C java 

clean:
	@make -C lcmtypes -s clean                                                 
	@make -C balancebot -s clean
	@make -C test_motors -s clean
	@make -C test_lcm -s clean
	@make -C optitrack/common -s clean                                         
	@make -C optitrack -s clean                                                
	@make -C java -s clean
