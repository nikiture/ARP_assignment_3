#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <sys/time.h>
#include <sys/select.h>
#include <unistd.h> 
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
#include "world_info.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

const int log_id = 0; //position in log_file where pid is written
struct position obs_pos [obs_num];
double x_size = MAP_X_SIZE;
double y_size = MAP_Y_SIZE;
char gen_id = 'o';

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


void generate_obstacles () {
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i].x = ((double) rand () / ((double) RAND_MAX)) * (x_size - 10) + 5;
        obs_pos [i].y = ((double) rand () / ((double) RAND_MAX)) * (y_size - 10) + 5;
    }
}

void send_obstacle_to_server (int fd_out, char * out_msg, char * tmp_msg) {
    if (sprintf (out_msg, "%c%c%d%c", gen_id, '[', obs_num, ']') < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (sprintf (tmp_msg, "%s%.3lf%c%.3lf", out_msg, obs_pos [0].y, ',', obs_pos [0].x) < 0) {
        perror ("sprintf");
        exit (EXIT_FAILURE);
    }
    if (strcpy (out_msg, tmp_msg) < 0) {
        perror ("strcpy");
        exit (EXIT_FAILURE);
    }
    for (int i = 1; i < obs_num; i++) {
        if (sprintf (tmp_msg, "%s%c%.3lf%c%.3lf", out_msg, '|', obs_pos [i].y, ',', obs_pos [i].x) < 0) {
            perror ("sprintf");
            exit (EXIT_FAILURE);
        } 
        if (strcpy (out_msg, tmp_msg) < 0) {
            perror ("strcpy");
            exit (EXIT_FAILURE);
        }
    }
    if (write (fd_out, out_msg, strlen (out_msg) + 1) < 0) {
        perror ("write");
        exit (EXIT_FAILURE);
    }
    while (read (fd_out, tmp_msg, 1023) < 0) {
        if (errno != EINTR) {
            perror ("obstacle message response read");
            exit (EXIT_FAILURE);
        }
    }
}


int main (int argc, char ** argv) {
    struct sigaction sa;
    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = &watchdog_req;
    if (sigaction (SIGUSR1, &sa, NULL) < 0) {
        perror ("sigaction");
        exit (EXIT_FAILURE);
    }
    if (argc < 2) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    struct hostent * server;

    int port;
    int sockfd;

    server = gethostbyname (argv [1]);
    if (server == NULL) {
        fprintf (stderr, "No host with given name found\n");
        exit (EXIT_FAILURE);
    }

    sscanf (argv [2], "%d", &port);

    /*sock_fd = socket (AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror ("socket 1");
        exit (EXIT_FAILURE);
    }
    struct sockaddr_in server_addr, client;

    bzero ((char *) &server_addr, sizeof (server_addr));
    server_addr.sin_family = AF_INET;

    bcopy ((char *) server->h_addr, (char *) &server_addr.sin_addr.s_addr, server->h_length);
    server_addr.sin_port = htons (port);

    while (connect (sock_fd, (struct sockaddr *) &server_addr, sizeof (server_addr)) < 0) {
        if (errno != EINTR) {
            perror ("Connection error 1");
            exit (EXIT_FAILURE);
        }
    }*/

    /*int in_fd, out_fd;
    if (sscanf (argv [1], "%d %d", &in_fd, &out_fd) < 0) {
        perror ("sscanf");
        exit (EXIT_FAILURE);
    }*/
    char out_msg [1024], tmp_msg [1024];
    
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
    srand (time (NULL) + 100);
    struct timespec t; 
    int count = 0;
    int reset_msg, time_left, sec_passed;
    sigset_t select_mask;
    sigaddset (&select_mask, SIGUSR1);

    struct timeval t_start, t_curr;
    fd_set in_pipe;

    //blocking read waiting for the server to be ready and to receive map size
    /*char start_msg [20];
    while (read (in_fd, start_msg, 20) < 0) {
        if (errno != EINTR) {
            perror ("map size read");
            exit (EXIT_FAILURE);
        }
    }*/
    int sock_fd;
    sock_fd = socket (AF_INET, SOCK_STREAM, 0);
    if (sock_fd < 0) {
        perror ("socket 1");
        exit (EXIT_FAILURE);
    }
    struct sockaddr_in server_addr, client;

    bzero ((char *) &server_addr, sizeof (server_addr));
    server_addr.sin_family = AF_INET;

    bcopy ((char *) server->h_addr, (char *) &server_addr.sin_addr.s_addr, server->h_length);
    server_addr.sin_port = htons (port);

    printf ("obstacle: attempting connection to server\n");
    while (connect (sock_fd, (struct sockaddr *) &server_addr, sizeof (server_addr)) < 0) {
        if (errno != EINTR) {
            perror ("Connection error 1");
            exit (EXIT_FAILURE);
        }
    }
    printf ("obstacle: connection successful\n");
    sprintf (out_msg, "%s", "OI");
    if (write (sock_fd, out_msg, strlen (out_msg) + 1) < 0) {
        perror ("init message send");
        exit (EXIT_FAILURE);
    }
    while (read (sock_fd, tmp_msg, 1023) < 0) {
        if (errno != EINTR) {
            perror ("init message reception");
            exit (EXIT_FAILURE);
        }
    }
    printf ("obstacle: waiting for map size\n");
    while (read (sock_fd, tmp_msg, 1023) < 0) {
        if (errno != EINTR) {
            perror ("map size message reception");
            exit (EXIT_FAILURE);
        }
    }
    printf ("sending back map size\n");
    if (write (sock_fd, tmp_msg, strlen (tmp_msg) + 1) < 0) {
        perror ("map size message response");
        exit (EXIT_FAILURE);
    }
    printf ("obstacle: map size received\n");
    sscanf (tmp_msg, "%lf %lf", &y_size, &x_size);
    //sscanf (start_msg, "%lf %lf", &y_size, &x_size);

    //generate obstacles position
    generate_obstacles ();
    //send obstacle position to server
    send_obstacle_to_server (sock_fd, out_msg, tmp_msg);

    gettimeofday (&t_start, NULL);
    while (1) {
        t.tv_sec = 1;
        t.tv_nsec = 0;
        FD_ZERO (&in_pipe);
        FD_SET (sock_fd, &in_pipe);
        reset_msg = pselect (sock_fd + 1, &in_pipe, NULL, NULL, &t, &select_mask);
        if (reset_msg < 0) {
            perror ("pselect");
            exit (EXIT_FAILURE);
        }
        if (reset_msg > 0) {
            while (read (sock_fd, tmp_msg, 1024) < 0) {
                if (errno != EINTR) {
                    perror ("read");
                    exit (EXIT_FAILURE);
                }
            }
            if (tmp_msg [0] == 'q') {
                exit (EXIT_SUCCESS);
            }
            generate_obstacles ();

            send_obstacle_to_server (sock_fd, out_msg, tmp_msg);
        }
    }
    return 0;
}