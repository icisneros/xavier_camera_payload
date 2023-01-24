#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/time.h>

// Proof of concept for chrony synch to IMU PPS using file socket
// writes to a unix file socket created by chrony 
// To configure chrony add a line like:
// refclock SOCK /var/run/chrony.sock
// to /etc/chrony/chrony.conf
// TODO:
// * make socket non root
// * identify improved chrony delay and jitter parameters
// * check if this breaks rosbag tools (reindexing) or tf

#define SOCK_MAGIC 0x534f434b

struct sock_sample {
    struct timeval tv;
    double offset;
    int pulse;
    int leap;    /* notify that a leap second is upcoming */
    // cppcheck-suppress unusedStructMember
    int _pad;
    int magic;      /* must be SOCK_MAGIC */
};

typedef int socket_t;

/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
size_t strlcpy(char *dst, const char *src, size_t siz)
{
    size_t len = strlen(src);
    if (siz != 0) {
        if (len >= siz) {
            memcpy(dst, src, siz - 1);
            dst[siz - 1] = '\0';
        } else
            memcpy(dst, src, len + 1);
    }
    return len;
}


socket_t netlib_localsocket(const char *sockfile, int socktype)
/* acquire a connection to an existing Unix-domain socket */
{
    int sock;

    if ((sock = socket(AF_UNIX, socktype, 0)) < 0) {
        return -1;
    } else {
        struct sockaddr_un saddr;

        memset(&saddr, 0, sizeof(struct sockaddr_un));
        saddr.sun_family = AF_UNIX;
        (void)strlcpy(saddr.sun_path,
                      sockfile,
                      sizeof(saddr.sun_path));

        if (connect(sock, (struct sockaddr *)&saddr, SUN_LEN(&saddr)) < 0) {
            (void)close(sock);
            return -2;
        }
        return sock;
    }
}

class Chrony
{
 private:
  std::string chrony_path;
  int chronyfd;
 public:
  Chrony()
    {
      chronyfd=-1;
      chrony_path="/var/run/chrony.sock";
    };

  ~Chrony()
    {
      // close? shutdown?
      if (chronyfd>0)
	close(chronyfd);
    };

  void SetSocketName(const std::string name)
  {
    chrony_path=name;
  };

  bool Init()
  {
    if (access(chrony_path.c_str(), F_OK) != 0) {
      std::cout<<"PPS:chrony socket doesn't exist: " <<chrony_path<<"\n";
    } else {
      chronyfd = netlib_localsocket(chrony_path.c_str(), SOCK_DGRAM);
      if (chronyfd < 0)
	std::cout << "PPS: connect chrony socket failed: "<<chrony_path<<"\n";
    }
    return(chronyfd>0);
  };

  /* recieves timeval and offset in ns offset=pps-system */
  void Send( struct timeval tv, double offset )
  {
    
    /* chrony expects tv-sec since Jan 1970 */
    struct sock_sample sample;
    sample.pulse = 0;
    //sample.pulse = 1;
    sample.leap = 0; // no warning
    sample.magic = SOCK_MAGIC;
    /* chronyd wants a timeval that's just the top of the second */
    sample.tv = tv;
    sample.offset = offset;
    sample._pad = 0;
    
    if (chronyfd>0)
      (void)send(chronyfd, &sample, sizeof (sample), 0);
    else
      std::cout << "PPS: chrony socket not open: "<<chrony_path<<"\n";
     
  };
};
