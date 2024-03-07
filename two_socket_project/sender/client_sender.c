#include <stdio.h>
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/time.h>
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <ncurses.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "world_info.h"
#include <errno.h>
#include <netinet/in.h>
#include <netdb.h>
#define frequency 50

int logfd = 0;
int log_id = 0;
#define GEN_NUM 2
char gen_identifyer [GEN_NUM] = {'o', 't'};

void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {

        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            printf ("issues opening file\n");
        }

        int pid = getpid ();
        char logdata [10];
        
        if (sprintf (logdata, "%d", pid) < 0) {
            perror ("sprintf");
            printf ("issues formatting\n");
        }

        if (write (fd, logdata, sizeof (logdata)) < 0) {
            perror ("write");
            printf ("issues writing to file\n");
        }

        close (fd);
    }
}

void dump_pipe_content (int fd, fd_set * fds_in, struct timespec * t, sigset_t * select_mask, char * IO_msg) {
    int syscall_res = 0;
    do {
        FD_ZERO (fds_in);
        FD_SET (fd, fds_in);
        t->tv_sec = 0;
        t->tv_nsec = 0;
        syscall_res = pselect (fd + 1, fds_in, NULL, NULL, t, select_mask);
        if (syscall_res < 0) {
            perror ("pipe content dumpp select");
            exit (EXIT_FAILURE);
        }
        if (syscall_res > 0) {
            if (read (fd, IO_msg, 1024) < 0) {
                perror ("pipe content dump read");
                exit (EXIT_FAILURE);
            }
        }
    } while (syscall_res > 0);
}



int main (int argc,  char ** argv) {
    struct sigaction sa;
    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = &watchdog_req;
    if (sigaction (SIGUSR1, &sa, NULL) < 0) {
        perror ("sigaction");
        exit (EXIT_FAILURE);
    }
    int fd = open (log_file [log_id], O_WRONLY);
    if (fd < 0) {
        perror ("log_file open");
    }
    int pid = getpid ();
    if (pid < 0) {
        perror ("pid reception");
        exit (EXIT_FAILURE);
    }

    char logdata [10];
    
    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("sprintf");
    }
    

    if (write (fd, logdata, sizeof (logdata)) < 0) {
        perror ("write");

    }

    if (close (fd) < 0) {
        perror ("close");
    }


    int obs_fd_in, obs_fd_out;
    int targ_fd_in, targ_fd_out;
    struct hostent * server;

    int port;

    server = gethostbyname (argv [1]);
    if (server == NULL) {
        fprintf (stderr, "No host with given name found\n");
        exit (EXIT_FAILURE);
    }

    sscanf (argv [2], "%d", &port);

    int gen_fds_in [GEN_NUM], gen_fds_out [GEN_NUM];
    int maxfd = -1;
    for (int i = 0; i < GEN_NUM; i++) {
        sscanf (argv [3 + i], "%d %d", &(gen_fds_in [i]), &(gen_fds_out [i]));
        if  (maxfd < gen_fds_in [i]) {
            maxfd = gen_fds_in [i];
        }
    }

    int sock_fd1, sock_fd2;
    int targ_sock_fd, obs_sock_fd;

    sock_fd1 = socket (AF_INET, SOCK_STREAM, 0);
    if (sock_fd1 < 0) {
        perror ("socket 1");
        exit (EXIT_FAILURE);
    }
    struct sockaddr_in server_addr, client;

    bzero ((char *) &server_addr, sizeof (server_addr));
    server_addr.sin_family = AF_INET;

    bcopy ((char *) server->h_addr, (char *) &server_addr.sin_addr.s_addr, server->h_length);
    server_addr.sin_port = htons (port);

    while (connect (sock_fd1, (struct sockaddr *) &server_addr, sizeof (server_addr)) < 0) {
        if (errno != EINTR) {
            perror ("Connection error 1");
            exit (EXIT_FAILURE);
        }
    }
    double x_size, y_size; 
    char IO_msg [1024], tmp_msg [1023];
    char reset_msg [3];
    sprintf (reset_msg, "%s", "GE");
    char stop_msg [5];
    sprintf (stop_msg, "%s", "STOP");

    /*start of connection: send of message TI and OI and reception of map sizes*/
    sprintf (IO_msg, "%s", "OI");
    if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("init socket write");
        exit (EXIT_FAILURE);
    }
    while (read (sock_fd1, IO_msg, 2) < 0) {
        if (errno != EINTR) {
            perror ("init socket read");
            exit (EXIT_FAILURE);
        }
    }
    while (read (sock_fd1, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("map size read");
            exit (EXIT_FAILURE);
        }
    }
    if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("map size response");
        exit (EXIT_FAILURE);
    }
    printf ("connection successful\nsending map dimensions to the generator processes\n");
    for (int i = 0; i < GEN_NUM; i++) {
        while (write (gen_fds_out [i], IO_msg, strlen (IO_msg) + 1) < 0) {
            if (errno != EINTR) {
                perror ("map size write");
                exit (EXIT_FAILURE);
            }
        }
    }
    
    sprintf (IO_msg, "%s", "TI");
    if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("init socket write 2");
        exit (EXIT_FAILURE);
    }
    while (read (sock_fd1, IO_msg, 2) < 0) {
        if (errno != EINTR) {
            perror ("init socket read 2");
            exit (EXIT_FAILURE);
        }
    }
    while (read (sock_fd1, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("map size read 2");
            exit (EXIT_FAILURE);
        }
    }
    if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("map size response 2");
        exit (EXIT_FAILURE);
    }

    struct timespec t;

    fd_set fds_in;
    int syscall_res;
    int req_idx, sel_idx;

    int rand_selector;

    int simul_req [GEN_NUM];

    sigset_t select_mask;
    if (sigemptyset (&select_mask) < 0) {
        perror ("sigmask set 1");
        exit (EXIT_FAILURE);
    }

    if (sigaddset (&select_mask, SIGUSR1) < 0) {
        perror ("sigmask set 2");
        exit (EXIT_FAILURE);
    }

    srand (time (NULL));

    while (1) {
        t.tv_sec = 0;
        t.tv_nsec = 1000 * SEC_TO_USEC / frequency;
        //pselect on input socket
        FD_ZERO (&fds_in);
        FD_SET (sock_fd1, &fds_in);
        syscall_res = pselect (sock_fd1 + 1, &fds_in, NULL, NULL, &t, &select_mask);
        if (syscall_res < 0) {
            perror ("socket select");
            exit (EXIT_FAILURE);
        }
        //if message from socket check if GE or STOP message
        if (syscall_res > 0) {
            //printf ("message from socket found\n");
            if (read (sock_fd1, IO_msg, 1024) < 0) {
                perror ("socket read");
                exit (EXIT_FAILURE);
            }
            printf ("message from socket received\n");
            if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
                perror ("socket response");
                exit (EXIT_FAILURE);
            }
            //if GE new target and obstacle generation
            if (strcmp (reset_msg, IO_msg) == 0) { 
                printf ("reset messsage\n");
                for (int i = 0; i < GEN_NUM; i++) {
                    dump_pipe_content (gen_fds_in [i], &fds_in, &t, &select_mask, IO_msg);
                    if (dprintf (gen_fds_out [i], "%c", 'r') < 0) {
                        perror ("reset generators write");
                        exit (EXIT_FAILURE);
                    }
                }
            }
            //if STOP closure of programs
            else if (strcmp (stop_msg, IO_msg) == 0) { 
                printf ("stop message received\n");
                for (int i = 0; i < GEN_NUM; i++) {
                    dprintf (gen_fds_out [i], "%c", 'q');
                }
                sprintf (logdata, "%c", 'q');
                write (logfd, logdata, strlen (logdata) + 1);
                sleep (3);
                usleep (100000);//sleep should be interrupted because of the watchdog monitoring, after that it waits a little bit more to let the watchdog notice the quit message on the logpipe
                close (logfd);
                printf ("quit message sent to watchdog, exiting\n");
                exit (EXIT_SUCCESS);
            }
        }
        
        

        //check if messages from generator processes
        FD_ZERO (&fds_in);
        for (int i = 0; i < GEN_NUM; i++) {
            FD_SET (gen_fds_in [i], &fds_in);
        }
        t.tv_sec = 0;
        t.tv_nsec = 0;
        syscall_res = pselect (maxfd + 1, &fds_in, NULL, NULL, &t, &select_mask);
        switch (syscall_res) {
            case -1://error on select
                perror ("pselect");
                exit (EXIT_FAILURE);
                break;
            case 0://no requests sent to server
                break;
            case 1: //only one request: no need to randomly choose but need to identify requester (its id) for proper action (fd- and data transmission-wise)
                //find which process has written
                for (req_idx = 0; req_idx < GEN_NUM; req_idx ++) {
                    if (FD_ISSET (gen_fds_in [req_idx], &fds_in)) break;
                }
                //read message from pipe
                syscall_res = read (gen_fds_in [req_idx], tmp_msg, sizeof (tmp_msg));
                if (syscall_res < 0) {
                    perror ("read");
                    
                    for (int i = 0; i < GEN_NUM; i++) {
                        close (gen_fds_in [i]);
                        close (gen_fds_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                printf ("message read, redirecting to remote server\n");
                //add proper generator  identifyer at the beginning of the message string and then write it
                sprintf (IO_msg, "%c%s", gen_identifyer [req_idx], tmp_msg);
                if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
                    perror ("socket write");
                    exit (EXIT_FAILURE);
                }

                while (read (sock_fd1, IO_msg, 1024) < 0) {
                    if (errno != EINTR) {
                        perror ("socket response read");
                        exit (EXIT_FAILURE);
                    }
                }


                break;        
            default: //more than one process has pending request, random choice for fairness after identifying requesting processes
                printf ("multiple messages found, picking one to redirect\n");
                req_idx = 0;
                for (int i = 0; i < GEN_NUM ; i++) {//loop to store indexes (each one corresponding to a different process) in request queues
                    if (FD_ISSET (gen_fds_in [i], &fds_in)) {
                        simul_req [req_idx] = i;
                        req_idx++;
                    }
                }
                //take random index between the ones which have a request to process
                rand_selector = rand ();
                if (rand < 0) {
                    perror ("rand");
                    exit (EXIT_FAILURE);
                }
                sel_idx = rand_selector % syscall_res;
                req_idx = simul_req [sel_idx];

                syscall_res = read (gen_fds_in [req_idx], tmp_msg, 240);
                if (syscall_res < 0) {
                    perror ("read");
                    for (int i = 0; i < GEN_NUM; i++) {
                        close (gen_fds_in [i]);
                        close (gen_fds_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                //add proper generator  identifyer at the beginning of the message string and then write it
                sprintf (IO_msg, "%c%s", gen_identifyer [req_idx], tmp_msg);
                if (write (sock_fd1, IO_msg, strlen (IO_msg) + 1) < 0) {
                    perror ("socket write");
                    exit (EXIT_FAILURE);
                }
                while (read (sock_fd1, IO_msg, 1024) < 0) {
                    if (errno != EINTR) {
                        perror ("socket response read");
                        exit (EXIT_FAILURE);
                    }
                }
                break;
        }
    }

    return 0;
}