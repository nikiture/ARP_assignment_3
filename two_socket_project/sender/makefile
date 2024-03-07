all: Master obs_gen targ_gen watchdog

clean: clean_binaries clean_logs

clean_binaries:
	rm build/Master build/watchdog build/obstacle_generator build/target_generator build/client_sender

clean_logs:
	rm build/log_results.txt

start_build: 
	mkdir build

Master: 
	gcc Master.c -o build/Master

watchdog: 
	gcc watchdog.c -o build/watchdog

obs_gen:
	gcc obstacle_generator.c -o build/obstacle_generator

targ_gen:
	gcc target_generator.c -o build/target_generator

sock_client:
	gcc client_sender.c -o build/client_sender


