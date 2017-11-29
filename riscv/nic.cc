#include "nic.h"
#include "sim.h"
#include <cassert>


nic_t::nic_t(sim_t *host_sim) {
  sim = host_sim;
  num_of_rec_buf_count = 0;
  pthread_mutex_init(&blade->lock, NULL);
  set_up();
}
char* nic_t::NIC_PORT_NAME(reg_t addr) {
  switch(addr) {
    case 0:
      return "NIC_SEND_REQ";
    case 8:
      return "NIC_RECV_REQ";
    case 16:
      return "NIC_SEND_COMP";
    case 18:
      return "NIC_RECV_COMP";
    case 20:
      return "NIC_COUNTS";
    case 24:
      return "NIC_MACADDR";
    default:
      nic_err("Invalid addr");
      return "Invalid addr";
  }
  /* should not reach */
  return true;
};


bool nic_t::load(reg_t addr, size_t len, uint8_t* bytes) {
  /* Only word-sized values accepted */
  assert(len == sizeof(reg_t));
  switch(addr) {
    case NIC_SEND_COMP:
      return send_comp_status(bytes);
      break;
    case NIC_RECV_COMP:
      return recv_comp_status(bytes);
      break;
    case NIC_COUNTS:
      return counts_info(bytes);
      break;
    case NIC_MACADDR:
      return mac_address(bytes);      
      break;
    default:
      if(addr % 8 != 0 || addr > NIC_PORT_LAST) {
        nic_err("Unrecognized load to NIC offset %ld\n", addr);      
      } else { 
        nic_err("Cannot load from %s\n", NIC_PORT_NAME(addr)); 
      }
      return false;
  }

  /* should not reach */
  return true;
}

bool nic_t::store(reg_t addr, size_t len, const uint8_t* bytes) {
  /* Only word-sized values accepted */
  assert(len == sizeof(reg_t));
  switch(addr) {
    case NIC_SEND_REQ:
      return send(bytes, len);
      break;
    case NIC_RECV_REQ:
      return receive(bytes);
      break;
    default:
      if(addr % 8 != 0 || addr > NIC_PORT_LAST) {
        nic_err("Unrecognized store to NIC offset %ld\n", addr);      
      } else {
        nic_err("Cannot store to %s\n", NIC_PORT_NAME(addr)); 
      }
      return false;
  }

  /* should not reach */
  return true;
}

bool nic_t::send_comp_status(uint8_t* bytes) {

}

bool nic_t::recv_comp_status(uint8_t* bytes) {

}

bool nic_t::counts_info(uint8_t* bytes) {
  unsigned int send_req_avail = 0;
  unsigned int recv_req_avail = num_of_rec_buf_count;
  unsigned int send_comp_avail = 0;
  unsigned int rec_comp_avail = 0; 
  
}

bool nic_t::mac_address(uint8_t* bytes) {
  memcpy(bytes, interface_mac.ifr_hwaddr.sa_data, 6);
  return true;
}

bool nic_t::send(const uint8_t* bytes, size_t len) {

}

bool nic_t::receive(const uint8_t* bytes) {

}

bool nic_t::set_up() {
  memset(interface_name, 0, IFNAMSIZ);
  strcpy(interface_name, DEFAULT_INTERFACE_NAME); 
  if ((socket_fd = socket(AF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
    perror("socket");
    return false;
  }
  // Get index of interface to send on by name.
  memset(&interface_index, 0, sizeof(struct ifreq));
  strncpy(interface_index.ifr_name, interface_name, IFNAMSIZ - 1);
  if (ioctl(socket_fd, SIOCGIFINDEX, &interface_index) < 0) {
    perror("SIOCGIFINDEX");
    return false;
  }
  //What's this part is doing?
  socket_address.sll_ifindex = interface_index.ifr_ifindex;
  socket_address.sll_halen = ETH_ALEN;  // Ethernet address length.
  // TODO(growly): What's this for?
  for (int i = 0; i < ETH_ALEN; ++i) {
    blade->socket_address.sll_addr[i] = dst_mac_address[i];
  }

  // Get MAC of interface to send on, by name.
  memset(&interface_mac, 0, sizeof(struct ifreq));
  strncpy(interface_mac.ifr_name, interface_name, IFNAMSIZ - 1);
  if (ioctl(socket_fd, SIOCGIFHWADDR, &interface_mac) < 0) {
    perror("SIOCGIFHWADDR");
    return false;
  }
  
  printf("Attatched to Network Interface.\n"
         "\tinterface: %s\n",
         interface_name);
  
  keep_listening = true;
  
  // Ok now spawn a receiving thread.
  if (pthread_create(&receive_thread,
                     NULL,
                     &listen_for_protocol,
                     NULL)) {
    fprintf(stderr, "Error: Couldn't create remote memory receive thread.\n");
    return false;
  }
  
  // Init is not complete until the listener thread has signalled its readiness.
  pthread_mutex_lock(&lock);
  while (!listener_up) {
    pthread_cond_wait(&listener_up_cond, &lock);
  }
  pthread_mutex_unlock(&lock);
  printf("Init complete.\n");
  return true;
}

void *listen_for_protocol(void *something) {
  printf("Protocol listener up.\n");
  pthread_mutex_lock(lock);
  char *interface_name = interface_name;
  struct ifreq interface_index = interface_index;
  struct ifreq interface_mac = interface_mac;
  pthread_mutex_unlock(&blade->lock);

  struct ifreq if_ip;
  struct sockaddr_storage remote_addr;
  char buffer[RECEIVE_BUFFER_SIZE_BYTES];
  memset(buffer, 0, RECEIVE_BUFFER_SIZE_BYTES);

  rx_socket_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (rx_socket_fd == -1) {
    perror("Listener socket");
    return NULL;
  }

  int status = 0;

  int sockopt = 1;
  status = setsockopt(rx_socket_fd,
                      SOL_SOCKET,
                      SO_REUSEADDR,
                      &sockopt,
                      sizeof(sockopt));
  if (status < 0) {
    perror("setsockopt");
    close(rx_socket_fd);
    return NULL;
  }

  status = setsockopt(rx_socket_fd,
                      SOL_SOCKET,
                      SO_REUSEPORT,
                      &sockopt,
                      sizeof(sockopt));
  if (status < 0) {
    perror("setsockopt");
    close(rx_socket_fd);
    return NULL;
  }

  struct sockaddr_ll ll_addr;
  ll_addr.sll_family = PF_PACKET;
  ll_addr.sll_ifindex = interface_index.ifr_ifindex;
  ll_addr.sll_protocol = htons(ETH_P_ALL);
  status = bind(rx_socket_fd, (struct sockaddr*)&ll_addr,
                sizeof(struct sockaddr_ll));

  // TODO(growly): This was the alternate way:
  //printf("setsockopt: SO_BINDTODEVICE\n");
  //status = setsockopt(blade->rx_socket_fd,
  //                    SOL_SOCKET,
  //                    SO_BINDTODEVICE,
  //                    interface_name,
  //                    strlen(interface_name) + 1);
  if (status < 0) {
    perror("bind failed\n");
    close(rx_socket_fd);
    return NULL;
  }

  // Now that the socket is up, but just before we block to listen, we signal
  // that we're ready.

  pthread_mutex_lock(&lock);
  listener_up = TRUE;
  pthread_cond_signal(&listener_up_cond);
  pthread_mutex_unlock(&lock);

  while (blade->keep_listening) {
    // TODO(growly): This needs a timeout so that termination is graceful.
    pthread_mutex_lock(&lock);
    int num_bytes = recvfrom(rx_socket_fd,
                             rx_buffers[num_of_rec_buf_count++],
                             RECEIVE_BUFFER_SIZE_BYTES,
                             0,
                             NULL,
                             NULL);
    pthread_mutex_unlock(&lock);
  }
  close(rx_socket_fd);
}
