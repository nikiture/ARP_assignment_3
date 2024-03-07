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
int log_id = 3;
#define frequency 50



void watchdog_req (int signumb) {
    if (signumb == SIGUSR1) {

        int fd = open (log_file [log_id], O_WRONLY);
        if (fd < 0) {
            perror ("log_file open");
            printf ("issues opening file\n");
        }

        int pid = getpid ();
        char logdata [10];
        
        if (sprintf (logdata, "%c", 'r') < 0) {
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

int main (int argc, char ** argv) {
    //alias for both obstacle generator and target generator on receiving end of the two machine program
    struct sigaction sa;
    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = &watchdog_req;
    if (sigaction (SIGUSR1, &sa, NULL) < 0) {
        perror ("sigaction");
        exit (EXIT_FAILURE);
    }


    int fd_wtch = open (log_file [log_id], O_WRONLY, 0666);
    if (fd_wtch < 0) {
        perror ("log pipe open");
        exit (EXIT_FAILURE);
    }

    char logdata [10];
    int pid = getpid ();
    //printf ("%d\n", pid);

    if (sprintf (logdata, "%d", pid) < 0) {
        perror ("pid formatting");
        exit (EXIT_FAILURE);
    }

    if (write (fd_wtch, logdata, sizeof (logdata)) < 0) {
        perror ("watchdog write");

        exit (EXIT_FAILURE);
    }

    int sock_fd, clilen, syscall_res, port;
    char IO_msg [1024], tmp_msg [1024];
    struct sockaddr_in serv_addr, client_addr;


    int serv_fd_in [2];
    int serv_fd_out [2];
    int maxfd = -4;

    for (int i = 0; i < 2; i++) {
        sscanf (argv [i + 2], "%d %d", &(serv_fd_in [i]), &(serv_fd_out [i]));
        if (maxfd < serv_fd_in [i]) maxfd = serv_fd_in [i];
    }
    printf ("%d %d\n", serv_fd_out [0], serv_fd_out [1]);


    char gen_id;

    int req_idx, sel_idx, rand_selector;
    int simul_req [2];

    sigset_t select_mask;
    if (sigemptyset (&select_mask) < 0) {
        perror ("sigmask set 1");
        exit (EXIT_FAILURE);
    }

    if (sigaddset (&select_mask, SIGUSR1) < 0) {
        perror ("sigmask set 2");
        exit (EXIT_FAILURE);
    }

    sock_fd = socket (AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror ("socket opening");
        exit (EXIT_FAILURE);
    }
    bzero (&serv_addr, sizeof (serv_addr));
    sscanf (argv [1], "%d", &port);

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons (port);
    serv_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind (sock_fd, (struct sockaddr *) &serv_addr, sizeof (serv_addr)) < 0) {
        perror ("socket binding");
        exit (EXIT_FAILURE);
    }

    listen (sock_fd, 1);
    clilen = sizeof (client_addr);

    int sock_num = 2;
    int new_sockfd, iterator = 0;
    do {
        new_sockfd = accept (sock_fd, (struct sockaddr *) &client_addr, &clilen); 
        if (new_sockfd < 0 && errno != EINTR) {
            perror ("accept");
            exit (EXIT_FAILURE);
        }
        if (new_sockfd > 0 && iterator != sock_num - 1) {
            if (fork () == 0) {
                printf ("socket server: connection created\n");
                break;
            } else new_sockfd = -10;
        }
        iterator ++;
    } while (new_sockfd < 0);
    printf ("%d\nsocket server: waiting for init message\n", new_sockfd);
    /*single writer single receiver version*/
    while (read (new_sockfd, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("init message read 1");
            exit (EXIT_FAILURE);
        }
    }
    if (write (new_sockfd, IO_msg, strlen (IO_msg)) < 0) {
        perror ("init message write 1");
        exit (EXIT_FAILURE);
    }

    usleep (400);
    printf ("socket server: sending map size\n");

    sprintf (IO_msg, "%.3lf %.3lf", MAP_Y_SIZE, MAP_X_SIZE);
    
    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("map size write 1");
        exit (EXIT_FAILURE);
    }
    printf ("socket server: waiting for response from client\n");
    while (read (new_sockfd, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("map size response 1");
            exit (EXIT_FAILURE);
        }
    }
    printf ("socket server: map size received successfully\nwaiting for obstacles/targets\n");
    /*while (read (new_sockfd, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("init message read 2");
            exit (EXIT_FAILURE);
        }
    }
    if (write (new_sockfd, IO_msg, strlen (IO_msg)) < 0) {
        perror ("init message write 2");
        exit (EXIT_FAILURE);
    }
    sprintf (IO_msg, "%.3lf %.3lf", MAP_Y_SIZE, MAP_X_SIZE);
    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
        perror ("map size write 2");
        exit (EXIT_FAILURE);
    }
    while (read (new_sockfd, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            perror ("map size response 2");
            exit (EXIT_FAILURE);
        }
    }*/

    fd_set in_fds;
    struct timespec t;

    while (1) {
        //check if message from socket 
        FD_ZERO (&in_fds);
        FD_SET (new_sockfd, &in_fds);
        t.tv_sec = 0;
        t.tv_nsec = 1000 * SEC_TO_USEC / frequency;
        syscall_res = pselect (new_sockfd + 1, &in_fds, NULL, NULL, &t, &select_mask);
        if (syscall_res < 0) {
            perror ("select");
            exit (EXIT_FAILURE);
        }

        //if message read, find the corresponding generator process and send to the appropriate fd
        if (syscall_res > 0) {
            printf ("found message in socket\n");
            if (read (new_sockfd, IO_msg, 1024) < 0) {
                perror ("socket message read");
                exit (EXIT_FAILURE);
            }
            if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
                perror ("socket response write");
                exit (EXIT_FAILURE);
            }
            printf ("socket response sent\n");
            sscanf (IO_msg, "%c%s", &gen_id, tmp_msg);
            if (gen_id == 'O' || gen_id == 'o') {
                printf ("new obstacles!\n");
                if (write (serv_fd_out [0], tmp_msg, strlen (tmp_msg) + 1) < 0) {
                    perror ("message redirection write obs");
                    exit (EXIT_FAILURE);
                }
            }
            else {
                printf ("new targets!\n");
                if (write (serv_fd_out [1], tmp_msg, strlen (tmp_msg) + 1) < 0) {
                    perror ("message redirection write targ");
                    exit (EXIT_FAILURE);
                }
            }
        }
        //check for message from the blackboard and send them (properly modified) to the socket 
        FD_ZERO (&in_fds);
        for (int i = 0; i < 2; i++) {
            FD_SET (serv_fd_in [i], &in_fds);
        }
        t.tv_sec = 0;
        t.tv_nsec = 0;
        syscall_res = pselect (maxfd + 1, &in_fds, NULL, NULL, &t, &select_mask);

        switch (syscall_res) {
            case -1://error on select
                perror ("pselect");
                exit (EXIT_FAILURE);
                break;
            case 0://no requests sent to server
                break;
            case 1: //only one request: no need to randomly choose but need to identify requester (its id) for proper action (fd- and data transmission-wise)
                //find which process has written
                printf ("message from blackboard\n");
                for (req_idx = 0; req_idx < 2; req_idx ++) {
                    if (FD_ISSET (serv_fd_in [req_idx], &in_fds)) break;
                }
                //read message from pipe
                syscall_res = read (serv_fd_in [req_idx], IO_msg, sizeof (IO_msg));
                if (syscall_res < 0) {
                    perror ("read");
                    
                    for (int i = 0; i < 2; i++) {
                        close (serv_fd_in [i]);
                        close (serv_fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }
                //execute adequate response to message received based on content of the message and which process wrote
                /*if (process_request (req_idx, gen_fds_out [req_idx], &drone_pos, obstacle_pos, targ_pos, IO_msg, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
                    perror ("process requets");
                    exit(EXIT_FAILURE);
                }*/
                /*if (dprintf (sock_fd1, "%c%s\n", gen_identifyer [req_idx], IO_msg) < 0) {
                    perror ("socket write");
                    exit (EXIT_FAILURE);
                }*/
                if (IO_msg [0] == 'r') {
                    printf ("reset message\n");
                    sprintf (IO_msg, "%s", "GE");
                    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
                        perror ("reset write");
                        exit (EXIT_FAILURE);
                    }
                    if (read (new_sockfd, IO_msg, 1024) < 0) {
                        perror ("reset response read");
                        exit (EXIT_FAILURE);
                    }
                    printf ("received socket respond for reset\n");
                    
                }
                else if (IO_msg [0] == 'q') {
                    printf ("stop message\n");
                    sprintf (IO_msg, "%s", "STOP");
                    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
                        perror ("quit write");
                        exit (EXIT_FAILURE);
                    }
                    if (read (new_sockfd, IO_msg, 1024) < 0) {
                        perror ("quit response read");
                        exit (EXIT_FAILURE);
                    }
                    printf ("message sent successfully through the socket, bye\n");
                    exit (EXIT_SUCCESS);
                }

                break;        
            default: //more than one process has pending request, random choice for fairness after identifying requesting processes
                req_idx = 0;
                for (int i = 0; i < 2 ; i++) {//loop to store indexes (each one corresponding to a different process) in request queues
                    if (FD_ISSET (serv_fd_in [i], &in_fds)) {
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

                syscall_res = read (serv_fd_in [req_idx], IO_msg, 240);
                if (syscall_res < 0) {
                    perror ("read");
                    
                    for (int i = 0; i < 2; i++) {
                        close (serv_fd_in [i]);
                        close (serv_fd_out [i]);
                    }
                    exit (EXIT_FAILURE);
                }

                /*if (process_request (req_idx, fd_out [req_idx], &drone_pos, obstacle_pos, targ_pos, IO_msg, answer_string, obs_changed, &obs_placed, targ_changed, &targ_placed, &targ_to_reach) < 0) {
                    perror ("process requets");
                    
                    exit(EXIT_FAILURE);
                }*/
                /*if (dprintf (sock_fd1, "%c%s\n", gen_identifyer [req_idx], IO_msg) < 0) {
                    perror ("socket write");
                    exit (EXIT_FAILURE);
                }*/
                if (IO_msg [0] == 'r') {
                    printf ("reset message\n");
                    sprintf (IO_msg, "%s", "GE");
                    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
                        perror ("reset write");
                        exit (EXIT_FAILURE);
                    }
                    if (read (new_sockfd, IO_msg, 1024) < 0) {
                        perror ("reset response read");
                        exit (EXIT_FAILURE);
                    }
                    printf ("received socket respond for reset\n");
                }
                else if (IO_msg [0] == 'q') {
                    printf ("stop message\n");
                    sprintf (IO_msg, "%s", "STOP");
                    if (write (new_sockfd, IO_msg, strlen (IO_msg) + 1) < 0) {
                        perror ("quit write");
                        exit (EXIT_FAILURE);
                    }
                    if (read (new_sockfd, IO_msg, 1024) < 0) {
                        perror ("quit response read");
                        exit (EXIT_FAILURE);
                    }
                    printf ("message sent successfully through the socket, bye\n");
                    exit (EXIT_SUCCESS);
                }
                break;
        } 
    }



    
    
    
    
    
    
    return 0;
}