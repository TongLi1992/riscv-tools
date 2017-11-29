#ifndef NIC_H
#define NIC_H
#include <queue>
#include "devices.h"
#include "encoding.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <netinet/ether.h>
#include <net/if.h>

#include <pthread.h>

#define NIC_SEND_REQ 0
#define NIC_RECV_REQ 8
#define NIC_SEND_COMP 16
#define NIC_RECV_COMP 18
#define NIC_COUNTS 20
#define NIC_MACADDR 24
#define NIC_PORT_LAST 24

#define nic_err(M, ...) fprintf(stderr, "SPIKE NIC: " M, ##__VA_ARGS__)

#define DEFAULT_INTERFACE_NAME "eth0"

#define RECEIVE_BUFFER_SIZE_BYTES 1500
#define NUM_OF_RECEIVE_BUFFER 16
/* Forward declare sim_t to avoid circular dep with sim.h */
class sim_t;

class nic_t : public abstract_device_t {
  public:
    nic_t(sim_t *host_sim);
    
    /* These are the standard load/store functions from abstract_device_t 
     * They get called when a registered address is loaded/stored */
    bool load(reg_t addr, size_t len, uint8_t* bytes);
    bool store(reg_t addr, size_t len, const uint8_t* bytes);
    bool set_up();
  private:
    char* NIC_PORT_NAME(reg_t addr);
    bool send_comp_status(uint8_t* bytes);
    bool recv_comp_status(uint8_t* bytes);
    bool counts_info(uint8_t* bytes);
    bool mac_address(uint8_t* bytes);
    bool send(const uint8_t* bytes, size_t len);
    bool receive(const uint8_t* bytes);
  private:
    sim_t *sim;
    char rx_buffers[NUM_OF_RECEIVE_BUFFER][RECEIVE_BUFFER_SIZE_BYTES];
    int num_of_rec_buf_count;
    char interface_name[IFNAMSIZ];
    int socket_fd;
    int rx_socket_fd;
    struct ifreq interface_index;
    struct ifreq interface_mac;
    struct sockaddr_ll socket_address;
    bool keep_listening;
    pthread_t receive_thread;
    int listener_up;
    pthread_cond_t listener_up_cond;
    pthread_mutex_t lock;
};
#endif
