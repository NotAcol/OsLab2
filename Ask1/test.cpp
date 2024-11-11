#include <fcntl.h>
#include <glob.h>
#include <grp.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <poll.h>
#include <pthread.h>
#include <pwd.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cctype>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ostream>
#include <string>

bool terminate{};

void SigAction(int signal, siginfo_t *info, void *);
int GetNum(char *buf, int &i, int n);

int main(int argc, char *argv[]) {
    //-----------------------some cleanup-------------------------------------
    glob_t pglob{};
    glob("/tmp/riddle-*", GLOB_NOESCAPE, NULL, &pglob);
    for (int i{}; i < pglob.gl_pathc; ++i) {
        unlink(pglob.gl_pathv[i]);
    }
    globfree(&pglob);
    unlink("./riddle.savegame");
    //----------------------------------------------

    // NOTE(acol): making read only file (0) (1)
    creat("./.hello_there", S_IRUSR);

    // NOTE(acol): unpause with SIGCONT (2)
    struct sigaction act{};
    act.sa_sigaction = SigAction;
    act.sa_flags = SA_RESTART | SA_SIGINFO | SA_NOCLDWAIT;
    if (sigemptyset(&act.sa_mask)) return 1;
    if (sigaction(SIGCHLD, &act, NULL)) return 1;

    // NOTE(acol): environment variables for (3) and extra hints
    setenv("ANSWER", "42", 0);
    setenv("I_NEED_TECH_HINTS", "1", 0);

    // NOTE(acol): fifo for 4 with rw-r--r-- perms
    mkfifo("./magic_mirror", 0644);

    // NOTE(acol): creating an fd 99 (5)
    int fd = open("./test.cpp", O_APPEND);
    dup2(fd, 99);

    // NOTE(acol): making ping pong pipes (6)
    int pingfd[2];
    pipe(pingfd);
    dup2(pingfd[0], 33);
    dup2(pingfd[1], 34);

    int pongfd[2];
    pipe(pongfd);
    dup2(pongfd[0], 53);
    dup2(pongfd[1], 54);

    // NOTE(acol): hard link .hello_there to .hey_there (7)
    link("./.hello_there", "./.hey_there");

    // NOTE(acol):  make sparce files bf00 to bf09 with user read write perms
    //              and writing 16 chars at 1073741824 for each one (8)
    char buf1[]{"aaaaaaaaaaaaaaa"};
    std::string name{"bf0"};
    for (int i{}; i < 10; ++i) {
        fd = open((name + std::to_string(i)).c_str(), O_WRONLY | O_CREAT,
                  S_IRUSR | S_IWUSR);
        lseek(fd, 1073741824, SEEK_SET);
        write(fd, buf1, 16);
        close(fd);
    }

    // NOTE(acol):  making an tcp server side for (9)
    int sd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    struct sockaddr_in sin;
    sin.sin_family = AF_INET;
    // NOTE(acol):  have to use same port like ssh, how do you dodge that with
    // urls
    sin.sin_port = htons(49842);
    sin.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
        std::cout << "failed to bind" << std::endl;
        return 1;
    }

    listen(sd, 1);

    // NOTE(acol): for (10) and (11)
    creat("./secret", S_IRUSR | S_IWUSR);
    link("secret", "secret_number");

    char buf[200];
    int secret_fd;

    int child_sdin_pipe[2];
    pipe(child_sdin_pipe);
    pid_t cpid{};

#ifdef GOOFY_PID
    // NOTE(acol): (14) OMEGA hacky way of forcing pid at least on arch, it
    // needs to be chown root tho :)
    secret_fd = open("/proc/sys/kernel/ns_last_pid", O_RDWR | O_CREAT, 0644);
    if (secret_fd <= 0) printf("cant open ns_last_pid\n");
    if (flock(secret_fd, LOCK_EX)) printf("cant flock ns_last_pid");
    sprintf(buf, "%d", 32766);
    write(secret_fd, buf, 6);
    cpid = fork();
#endif  // GOOFY_PID

#ifndef GOOFY_PID
    // NOTE(acol): (14) spam forks till we get to 32767, cba doing checks for
    // this just launch it after reboot
    while ((cpid = fork()) != 32767) {
        if (getpid() == 32767)
            break;
        else if (getpid() != 32767 && cpid == 0) {
            exit(0);
        }
        usleep(1000);
    }
#endif  // !GOOFY_PID

    if (!cpid) {
#ifdef GOOFY_PID
        // NOTE(acol): hacky way
        setgid(getgrnam("acol")->gr_gid);
        setuid(getpwnam("acol")->pw_uid);
#endif  // GOOFY_PID

        // NOTE(acol): making this pipe the stdin for child
        dup2(child_sdin_pipe[0], STDIN_FILENO);
        close(child_sdin_pipe[1]);
        execl("./riddle", "./riddle");
    } else {
        close(child_sdin_pipe[0]);
        // FIX(acol): CHILD PROROCESS DOESNT RAISE SIGCHLD WHEN IT PAUSES
        //            why????

        printf("\nCPID IS: %d\n", cpid);

#ifdef GOOFY_PID
        flock(secret_fd, LOCK_UN);
        close(secret_fd);
#endif  // DEBUG
        sleep(1);
        kill(cpid, SIGCONT);

        // NOTE(acol): putting accept from (9) here cause it's blocking
        struct sockaddr_in peer_addr;
        socklen_t peer_addr_len;
        int num1{}, num2{}, i{};
        char op{};

        int sockfd = accept(sd, (struct sockaddr *)&peer_addr, &peer_addr_len);
        if (sockfd < 0) printf("\nDIDNT ACCEPT ANYTHING FOR 9\n");
        int n = read(sockfd, buf, 200);
        for (; i < n && !isdigit(buf[i]); ++i) {
        }  // find first num

        for (; i < n && num2 == 0; ++i) {
            if (buf[i] == '+' || buf[i] == '-' || buf[i] == '/' ||
                buf[i] == '*') {
                op = buf[i];
            } else if (isdigit(buf[i])) {
                if (num1 == 0) {
                    num1 = GetNum(buf, i, n);
                } else {
                    num2 = GetNum(buf, i, n);
                    switch (op) {
                        case '+': {
                            num1 += num2;
                        } break;
                        case '-': {
                            num1 -= num2;
                        } break;
                        case '*': {
                            num1 *= num2;
                        } break;
                        case '/': {
                            num1 /= num2;
                        } break;
                    }
                }
            }
        }
        write(sockfd, std::to_string(num1).c_str(),
              std::to_string(num1).length());

        sleep(1);
        memset(buf, 0, sizeof(buf));
        secret_fd = open("./secret", O_RDONLY);
        std::string secret_answer;
        i = 0;
        read(secret_fd, buf, sizeof(buf));
        close(secret_fd);
        secret_fd = -1;
        for (; i < 100 && buf[i] != ':'; ++i) {
        }
        i += 2;
        while (i < 100 && buf[i] != '\n') {
            secret_answer += buf[i++];
        }
        std::cout << secret_answer << std::endl;
        secret_answer += '\n';
        write(child_sdin_pipe[1], secret_answer.c_str(),
              (int)(secret_answer.size()));

        // (11)
        while ((secret_fd = open("secret_number", O_RDONLY)) < 0);

        memset(buf, 0, sizeof(buf));
        secret_answer.clear();
        i = 0;
        usleep(100);
        int r = read(secret_fd, buf, sizeof(buf));
        for (; i < 100 && buf[i] != ':'; ++i) {
        }
        i += 2;
        while (i < 100 && buf[i] != '\n') {
            secret_answer += buf[i++];
        }
        secret_answer += '\n';
        std::cout << secret_answer;
        write(child_sdin_pipe[1], secret_answer.c_str(),
              (int)(secret_answer.size()));
        close(secret_fd);

        memset(buf, 0, sizeof(buf));
        chmod(".hello_there", 0644);
        usleep(10);
        //  NOTE(acol): (12) finding and mapping the tmp file with glob, the
        //  offset is the last 3 digits of the hex address
        printf("Give char: ");
        char symbol{};
        scanf("%c", &symbol);

        glob("/tmp/riddle-*", GLOB_NOESCAPE, NULL, &pglob);
        secret_fd = open(pglob.gl_pathv[0], O_RDWR);
        void *ptr =
            mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, secret_fd, 0);
        *((char *)(ptr) + 111) = symbol;

        close(secret_fd);
        munmap(ptr, 4096);
        globfree(&pglob);

        // NOTE(acol): (13) opening the file and truncating it into something
        // way bigger so it doesnt sigbus
        sleep(2);
        secret_fd = open(".hello_there", O_RDWR);
        ptr =
            mmap(NULL, 20480, PROT_READ | PROT_WRITE, MAP_SHARED, secret_fd, 0);

        sleep(1);
        ftruncate(secret_fd, 20480);

        write(child_sdin_pipe[1], "\n", 1);
        close(secret_fd);
        munmap(ptr, 20480);
        std::cout << "finished 13" << std::endl;
        usleep(100);

        for (int i{}; i < 10; ++i) {
            sleep(10);
        }

        for (int i{}; i < 2; ++i) {
            close(pingfd[i]);
            close(pongfd[i]);
        }
        close(sd);
        close(sockfd);

        name = "bf0";
        for (int i{}; i < 10; ++i) {
            unlink((name + std::to_string(i)).c_str());
        }

        unlink("./magic_mirror");
        unlink("./.hey_there");
        unlink("./.hello_there");
        unlink("./secret");
        kill(cpid, SIGINT);

        return 0;
    }
}

void SigAction(int signal, siginfo_t *info, void *) {
    if (info->si_code == CLD_STOPPED) {
        std::cout << "child paused\n";
        if (kill(info->si_pid, SIGCONT)) raise(SIGTERM);
    } else if (info->si_code == CLD_CONTINUED) {
        std::cout << "resumed\n";
    } else if (info->si_code == CLD_EXITED) {
        // terminate = true;
    }
}

int GetNum(char *buf, int &i, int n) {
    int answer{};
    for (; i < n && isdigit(buf[i]); ++i) {
        answer = answer * 10 + buf[i] - '0';
    }
    return answer;
}
