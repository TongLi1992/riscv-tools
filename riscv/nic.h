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
#include <sys/ioctl.h>
#include <pthread.h>
#include <unistd.h>
#include <list>
#include <utility>

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
#define NUM_OF_RECEIVE_BUFFER 15
#define NUM_OF_SEND_BUFFER 15
/* Forward declare sim_t to avoid circular dep with sim.h */
class sim_t;

class nic_t : public abstract_device_t {
  public:
    nic_t(sim_t *host_sim);
    ~nic_t(); 
    /* These are the standard load/store functions from abstract_device_t 
     * They get called when a registered address is loaded/stored */
    bool load(reg_t addr, size_t len, uint8_t* bytes);
    bool store(reg_t addr, size_t len, const uint8_t* bytes);
    bool set_up();
  private:
    const char* NIC_PORT_NAME(reg_t addr);
    bool send_comp(uint8_t* bytes);
    bool recv_comp(uint8_t* bytes);
    bool counts_info(uint8_t* bytes);
    bool mac_address(uint8_t* bytes);
    bool send(const uint8_t* bytes, size_t len);
    bool post_receive(const uint8_t* bytes, size_t len);
    void *listen_for_protocol();
    static void* staticFunction(void* p);
  private:
    sim_t *_sim;
    //pair(int:length of data received, const unsigned char*:address of the recived data stored)
    std::vector<std::pair<int, uintptr_t>> _recv_buffer;    
    char *_real_recv_buffer;
    unsigned int _recv_slot_count;
    unsigned int _send_slot_count;
    unsigned int _recv_comp_count;
    unsigned int _send_comp_count;
    char _interface_name[IFNAMSIZ];
    int _send_socket_fd;
    int _rx_socket_fd;
    struct ifreq _interface_index;
    struct ifreq _interface_mac;
    bool _keep_listening;
    pthread_t _receive_thread;
    bool _listener_up;
    pthread_cond_t _listener_up_cond;
    pthread_mutex_t _lock;
};
#endif
