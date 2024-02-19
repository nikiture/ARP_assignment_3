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

#include "world_info.h"
#include <errno.h>

#define DRONE_MARKER 'X'
#define OBSTACLE_MARKER 'O'
char Targ_marker [targ_num] [2];  //each target has up to two digits identifying it

const double framerate = 50;
int logfd = 0;


void printerror (const char * errmsg) {
    endwin ();
    perror (errmsg);
    if (logfd > 0) close (logfd);
    int time_left = sleep (5);
    while (time_left > 0) {
        time_left = sleep (time_left);
    }
    exit (EXIT_FAILURE);
}
const int log_id = 1; //position in log_file where pid is written

        
void watchdog_req (int signumb) {

    if (signumb == SIGUSR1) {

        logfd = open (log_file [log_id], O_WRONLY, 0666);
        if (logfd < 0) {
            printerror ("log file open");
        }

        char logdata [10];

        if (sprintf (logdata, "%d", getpid ()) < 0) {
            printerror ("log formatting");
        }
        if (write (logfd, logdata, sizeof (logdata)) < 0) {
            printerror ("log write");
        }
        close (logfd);

    }
}

void read_drone_from_BB (int in_fd, int out_fd, struct position * drone_pos, char * IO_msg, struct timespec * t, sigset_t * sigmask) {

    sprintf (IO_msg, "%c", 'd');

    if (write (out_fd, IO_msg, strlen (IO_msg) + 1) < 0) {
        printerror ("BB write");
        exit (EXIT_FAILURE);
    }

    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    int syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect drone");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 1024) < 0) {
        printerror ("BB read");
        exit (EXIT_FAILURE);
    }

    if (sscanf (IO_msg, "%lf %lf", &(drone_pos->x), &(drone_pos->y)) < 0) {
        printerror ("drone deformatting");
    }

    //adjustemnts to drone position to match with ncurses: positive going downward, maximum at 0
    drone_pos->y *= -1; 
    drone_pos->y += MAP_Y_SIZE;

    //limitations to positions put just in case to still visualize the drone, but they should not be necessary with the wall forces implemented
    if (drone_pos->x < 0) drone_pos->x = 0;
    if (drone_pos->x > MAP_X_SIZE) drone_pos->x = MAP_X_SIZE;
    if (drone_pos->y < 0) drone_pos->y = 0;
    if (drone_pos->y > MAP_Y_SIZE) drone_pos->y = MAP_Y_SIZE;
}

void request_obs (int in_fd, int out_fd, int * obs_placed, struct position obs_pos [obs_num], char * IO_msg, char * tmp_msg, struct timespec * t, sigset_t * sigmask) {

    if (sprintf (IO_msg, "%c", 'o') < 0) {
        printerror ("request formatting");
    }

    int syscall_res = write (out_fd, IO_msg, strlen (IO_msg) + 1);

    if (syscall_res < 0) {
        printerror ("request write");
    }

    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect obs");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 1024) < 0) {
        printerror ("request read");
    }

    if (IO_msg [0] == 'n') return; //obstacles not changed, message deformatting not to be done

    sscanf (IO_msg, "%*c%d%*c%s", obs_placed, tmp_msg); //in IO_msg are present the chars '[' ']' but not used by the process, %*c let the server match them without storing them
    strcpy (IO_msg, tmp_msg);

    if (*obs_placed > obs_num) *obs_placed = obs_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(obs_pos [0].x), &(obs_pos [0].y), tmp_msg);
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *obs_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(obs_pos [i].x), &(obs_pos [i].y), tmp_msg);
        strcpy (IO_msg, tmp_msg);
    }

    for (int i = 0; i < *obs_placed; i++) {
        obs_pos [i].y *= -1; 
        obs_pos [i].y += MAP_Y_SIZE;
    }
}


void request_targ (int in_fd, int out_fd, int * targ_placed, struct position targ_pos [targ_num], char * IO_msg, char * tmp_msg, struct timespec * t, sigset_t * sigmask) {
    if (sprintf (IO_msg, "%c", 't') < 0) {
        printerror ("request formatting");
    }

    int syscall_res = write (out_fd, IO_msg, strlen (IO_msg) + 1);

    if (syscall_res < 0) {
        printerror ("request write");
    }
    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (in_fd, &fd_input);
    syscall_res = pselect (in_fd + 1, &fd_input, NULL, NULL, NULL, sigmask);
    if (syscall_res < 0) printerror ("pselect obs");
    if (syscall_res == 0) return;
    if (read (in_fd, IO_msg, 1024) < 0) {
        printerror ("request read");
    }

    if (IO_msg [0] == 'n') return; //targets not changed, message deformatting not to be done

    sscanf (IO_msg, "%*c%d%*c%s", targ_placed, tmp_msg); //in IO_msg are present the chars '[' ']' used for the 3rd assignment format but not used by the process, %*c let the server match them without storing them
    strcpy (IO_msg, tmp_msg);

    if (*targ_placed > targ_num) *targ_placed = targ_num;

    sscanf (IO_msg, "%lf%*c%lf%s", &(targ_pos [0].x), &(targ_pos [0].y), tmp_msg);
    strcpy (IO_msg, tmp_msg);

    for (int i = 1; i < *targ_placed; i++) {
        sscanf (IO_msg, "%*c%lf%*c%lf%s", &(targ_pos [i].x), &(targ_pos [i].y), tmp_msg);
        strcpy (IO_msg, tmp_msg);
    }

    for (int i = 0; i < *targ_placed; i++) {
        targ_pos [i].y *= -1; 
        targ_pos [i].y += MAP_Y_SIZE;
    }
}

void request_current_target (int out_fd, int in_fd, int * curr_targ, char IO_msg [1024], int * reset, sigset_t * selectmask) {
    if (dprintf (out_fd, "%c", 'c') < 0) printerror ("current target request");

    while (read (in_fd, IO_msg, 1024) < 0) {
        if (errno != EINTR) {
            printerror ("current target read");
        }
    }
    
    if (sscanf (IO_msg, "%d", curr_targ) < 0) 
        printerror ("first target decoding");
}



int main (int argc, char ** argv) {
    struct sigaction sa;
    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = watchdog_req;
    if (sigaction (SIGUSR1, &sa, NULL) < 0) {
        perror ("sigaction");
        exit (EXIT_FAILURE);
    }

    for (int i = 0; i< targ_num; i++) {
        sprintf (Targ_marker [i], "%d", i);
    }
    int curr_targ = 0; //identifier of current target to reach

    int fd_in_server, out_fd_server;



    int count = 0;
    const int maxcount = 2;
    if (argc < 3) {
        printf ("not enough arguments\n");
        exit (EXIT_FAILURE);
    }
    sscanf (argv [1], "%d %d", &fd_in_server, &out_fd_server);
    
    
    int fd_drone;
    sscanf (argv [2], "%d", &fd_drone);

    int logfd = open (log_file [log_id], O_WRONLY, 0666);
    if (logfd < 0) printerror ("log file open");

    char logdata [10];
    int syscall_res;

    int pid = getpid ();
    if (pid < 0) printerror ("pid reception");

    if (sprintf (logdata, "%d", pid) < 0) 
        printerror ("pid log formatting");

    if (write (logfd, logdata, strlen (logdata) + 1) < 0) {
        printerror ("watchdog write");
    }

    struct position drone_pos;
    struct position obs_pos [obs_num];
    struct position targ_pos [targ_num];
    for (int i = 0; i < obs_num; i++) {
        obs_pos [i].x = 10;
        obs_pos [i].y = 10;
    }
    for (int i = 0; i < obs_num; i++) {
        targ_pos [i].x = 10;
        targ_pos [i].y = 10;
    }
    char IO_msg [1024], tmp_str [1024];
    int obs_placed = obs_num;
    int targ_placed = targ_num;
    

    int mapsize [2];
    int kb_res;
    sigset_t select_mask, orig_mask;

    if (sigemptyset (&select_mask) < 0) {
        printerror ("mask creation 1");
    }
    if (sigaddset (&select_mask, SIGUSR1) < 0) {
        printerror ("mask creation 2");
    }
    fd_set fd_input;
    FD_ZERO (&fd_input);
    FD_SET (fd_in_server, &fd_input);

    initscr(); 
    char startup_msg [3] [100] = {"welcome, here are the commands while waiting for the client to connect\n",
                                "use the w, e, r, s, d, f, x, c and v keys to control the drone\n",
                                "press q to quit the game, z to restart the game (placing the drone at the center of the map\n"};

    while (mvprintw (MAP_Y_SIZE / 2 - 1, 0, "%s", startup_msg [0]) < 0) {}
    while (mvprintw (MAP_Y_SIZE / 2, 0, "%s", startup_msg [1]) < 0) {}
    while (mvprintw (MAP_Y_SIZE / 2 + 1, 0, "%s", startup_msg [2]) < 0) {}
    
    refresh ();


    while (read (fd_in_server, IO_msg, 160) < 0) {
        if (errno != EINTR) printerror ("wait read");
    }

    syscall_res = clear ();
    if (syscall_res == ERR) {
        printf ("issues clearing\n\r");
        printerror ("addch clear");
    }
    refresh ();

    

    struct timespec t;
    int reset = 0;
    
    wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE);
    resizeterm (MAP_Y_SIZE, MAP_X_SIZE);
    cbreak();
    noecho();
    curs_set (0);
    nodelay (stdscr, 0);
    wtimeout (stdscr, 5);
    start_color ();
    if (has_colors () == 1) {
        if (init_color (COLOR_YELLOW, 1000, 568, 0) < 0) { //redefining COLOR_YELLOW to be orange
            printerror ("color_change");
        }
        init_pair (1, COLOR_BLUE, COLOR_BLACK); //first is characters, second is background
        init_pair (2, COLOR_WHITE, COLOR_BLACK);
        init_pair (3, COLOR_YELLOW, COLOR_BLACK);
        init_pair (4, COLOR_GREEN, COLOR_BLACK);

    }

    
    

    while (1) {

        //read kb presses and send them to dynamics process

        kb_res = getch ();


        if (sprintf (IO_msg, "%d", kb_res) < 0) {
            printerror ("drone sprintf");
        }
        if (write (fd_drone, IO_msg, strlen (IO_msg) + 1) < 0) {
            printerror ("drone write");
        }

        if (kb_res == 'q') {
            sprintf (logdata, "%c", 'q');
            //write to blackboard (which then writes to obstacle generator)
            write (out_fd_server, logdata, strlen (logdata) + 1);
            //write to watchdog and wait to ensure that the message is read
            write (logfd, logdata, strlen (logdata) + 1);
            sleep (3);
            usleep (50000);//sleep should be interrupted because of the watchdog monitoring, after that it waits a little bit more to let the watchdog notice the quit message on the logpipe
            close (logfd);
            exit (EXIT_SUCCESS);
        }
        if (kb_res == 'z') {
            if (sprintf (IO_msg, "%c", 'z') < 0) {
                printerror ("sprintf");
            }
            if (write (out_fd_server, IO_msg, strlen (IO_msg) + 1) < 0) {
                printerror ("write");
            }
            //read to wait for server acknowledgement (else risk of permanent block on other requests' read)
            while (read (fd_in_server, IO_msg, 160) < 0) {
                if (errno != EINTR) printerror ("BB reset response read");
            }
            curr_targ = 0;
        }

        /*update of the current target to be reached*/

        request_current_target (out_fd_server, fd_in_server, &curr_targ, IO_msg, &reset, &select_mask);

        if (curr_targ >= targ_placed) { //all targets reached, needed reset
            if (sprintf (IO_msg, "%c", 'r') < 0) {
                printerror ("sprintf");
            }
            if (write (out_fd_server, IO_msg, strlen (IO_msg) + 1) < 0) {
                printerror ("write");
            }
            if (sprintf (IO_msg, "%d", 'z') < 0) 
                printerror ("reset drone message encoding");
            if (write (fd_drone, IO_msg, strlen (IO_msg) + 1) < 0) 
                printerror ("reset drone write");

            //read to wait for server acknowledgement (else risk of permanent block on other requests' read)
            do {
                if (syscall_res = read (fd_in_server, IO_msg, 160) < 0) {
                    if (errno == EINTR) continue;
                    printerror ("BB reset response read");
                }
            } while (syscall_res < 0);
        
            curr_targ = 0;
            targ_placed = targ_num;
        }

        read_drone_from_BB (fd_in_server, out_fd_server, &drone_pos, IO_msg, &t, &select_mask);

        request_obs (fd_in_server, out_fd_server, &obs_placed, obs_pos, IO_msg, tmp_str, &t, &select_mask);

        request_targ (fd_in_server, out_fd_server, &targ_placed, targ_pos, IO_msg, tmp_str, &t, &select_mask);
        //printing stuff on terminal, blocking SIGUSR1 to avoid errors in the functions
        if (sigprocmask (SIG_SETMASK, &select_mask, &orig_mask) < 0) {
            printerror ("mask setting 1");
        }
        syscall_res = clear ();
        if (syscall_res == ERR) {
            printf ("issues clearing\n\r");
            printerror ("addch clear");
        }
        syscall_res = wresize (stdscr, MAP_Y_SIZE, MAP_X_SIZE); //makes sure the playground (visualized using box () below) is of the expected size
        if (syscall_res == ERR) {
            printf ("issues resizing\n\r");
            printerror ("resizing");
        }
        syscall_res = resize_term (MAP_Y_SIZE, MAP_X_SIZE);
        if (syscall_res == ERR) {
            printerror ("terminal resize");
        }

        refresh ();

        //set character color to white for border, then re-set it back to default    
        attron (COLOR_PAIR (2));
        box (stdscr, '|', '-');
        attroff (COLOR_PAIR (2));

        refresh ();

        attron (COLOR_PAIR (1));
        while (mvprintw ((int) round (drone_pos.y), (int) round (drone_pos.x), "%c", DRONE_MARKER) < 0) {
            if (errno != EINTR) printerror ("drone printing");
        }
        attroff (COLOR_PAIR (1));

        refresh ();

        attron (COLOR_PAIR (3)); 

        for (int i = 0; i < obs_placed; i++) {
            double x_pos = obs_pos [i].x;
            double y_pos = obs_pos [i].y;
            
            if (mvprintw ((int) round (y_pos), (int) round (x_pos), "%c", OBSTACLE_MARKER) < 0) {
            //if (syscall_res == ERR) { 
                //printf ("%d\n", i);
                sprintf (IO_msg, "%lf %lf, %d %s", obs_pos[i].y, y_pos, i, "obstacle placement");
                //printf ("%lf %lf\n", obs_pos [i].y, y_pos);
                printerror (IO_msg);/**/
            }
        }
        refresh ();
        
        attroff (COLOR_PAIR (3));

        attron (COLOR_PAIR (4));

        for (int i = curr_targ; i < targ_placed; i++) {
            if (mvprintw (round (targ_pos [i].y), round (targ_pos [i].x), "%s", Targ_marker [i]) < 0) {
                sprintf (IO_msg, "%lf %lf %d %s", targ_pos [i].x, targ_pos [i].y, i, "target placement");
                printerror (IO_msg);
            }
        }
        attroff (COLOR_PAIR (4));

        refresh ();
        if (sigprocmask(SIG_SETMASK, &orig_mask, &select_mask) < 0) {
            printerror ("mask unsetting");
        }

        usleep (SEC_TO_USEC / framerate);


    }
    return 0;
}