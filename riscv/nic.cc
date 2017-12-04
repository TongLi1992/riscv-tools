#include "nic.h"
#include "sim.h"
#include <cassert>

nic_t::nic_t(sim_t *host_sim) {
  _sim = host_sim;
  _recv_slot_count = NUM_OF_RECEIVE_BUFFER;
  _send_slot_count = NUM_OF_SEND_BUFFER;
  _recv_comp_count = 0;
  _send_comp_count = 0;
  pthread_mutex_init(&_lock, NULL);
  set_up();
  _real_recv_buffer = new char[RECEIVE_BUFFER_SIZE_BYTES];
}
const char* nic_t::NIC_PORT_NAME(reg_t addr) {
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
  return "should not reach";
};


bool nic_t::load(reg_t addr, size_t len, uint8_t* bytes) {
  /* Only word-sized values accepted */
  assert(len == sizeof(reg_t));
  switch(addr) {
    case NIC_SEND_COMP:
      return send_comp(bytes);
      break;
    case NIC_RECV_COMP:
      return recv_comp(bytes);
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
      return post_receive(bytes, len);
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

bool nic_t::send_comp(uint8_t* bytes) {
  pthread_mutex_lock(&_lock);
  _send_comp_count--;
  pthread_mutex_unlock(&_lock);
  return true;
}

bool nic_t::recv_comp(uint8_t* bytes) {
  pthread_mutex_lock(&_lock);
  //no need to worry about _recv_buffer is empty or _recv_comp_count is 0
  //because driver(icenet) of nic will make sure _recv_comp_count-- > 0 before calling this  
  _recv_buffer.erase(_recv_buffer.begin());
  _recv_comp_count--; 
  _recv_slot_count++;
  pthread_mutex_unlock(&_lock);
  return true;
}

bool nic_t::counts_info(uint8_t* bytes) {
  unsigned int send_req_count = _send_slot_count;
  unsigned int recv_req_count = _recv_slot_count << 4;
  unsigned int send_comp_count = _send_comp_count << 8;
  unsigned int recv_comp_count = _recv_comp_count << 12; 
  unsigned int result = send_req_count | recv_req_count | send_comp_count | recv_comp_count;
  memset(bytes, 0, 2);
  memcpy(bytes, &result, 2);
  return true;  
}

bool nic_t::mac_address(uint8_t* bytes) {
  memcpy(bytes, _interface_mac.ifr_hwaddr.sa_data, 6);
  return true;
}

bool nic_t::send(const uint8_t* bytes, size_t len) {
  assert(len == 8);
  pthread_mutex_lock(&_lock);
  _send_slot_count--; 
  pthread_mutex_unlock(&_lock);
  uintptr_t addr;
  memcpy(&addr, bytes, 6);
  unsigned int send_length;
  memcpy(&send_length, bytes+6, 2); 
  char *temp_buffer = (char*)malloc(send_length);
  memcpy(temp_buffer, (const void*)addr, send_length);
  //Todo: This should probabily use another thread to perform, since  wirte is blocking
  int status = write(_send_socket_fd, temp_buffer, send_length);
  if (status < 0) {
    printf("NIC Send failed.\n");
    return false;
  }  
  pthread_mutex_lock(&_lock); 
  _send_comp_count++;
  _send_slot_count++;
  pthread_mutex_unlock(&_lock);
  return true;
}

bool nic_t::post_receive(const uint8_t* bytes, size_t len) {
  assert(len == 8);
  pthread_mutex_lock(&_lock);
  uintptr_t addr;
  memcpy(&addr, bytes, 8);
  _recv_slot_count--;
  _recv_buffer.push_back(std::make_pair(-1, addr));
  pthread_mutex_unlock(&_lock);
  return true;
}

bool nic_t::set_up() {
  //Use eth0 as the interface name 
  memset(_interface_name, 0, IFNAMSIZ);
  strcpy(_interface_name, DEFAULT_INTERFACE_NAME); 

  //Create a RAW socket that allow us to send packets to network interface eth0
  if ((_send_socket_fd = socket(AF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
    perror("socket");
    return false;
  }
  
  // Get index of interface to send on by name.
  memset(&_interface_index, 0, sizeof(struct ifreq));
  strncpy(_interface_index.ifr_name, _interface_name, IFNAMSIZ - 1);
  if (ioctl(_send_socket_fd, SIOCGIFINDEX, &_interface_index) < 0) {
    perror("SIOCGIFINDEX");
    return false;
  }

  // Get MAC of interface to send on, by name.
  memset(&_interface_mac, 0, sizeof(struct ifreq));
  strncpy(_interface_mac.ifr_name, _interface_name, IFNAMSIZ - 1);
  if (ioctl(_send_socket_fd, SIOCGIFHWADDR, &_interface_mac) < 0) {
    perror("SIOCGIFHWADDR");
    return false;
  }
  
  printf("Attatched to Network Interface.\n"
         "\tinterface: %s\n",
         _interface_name);
  
  _keep_listening = true;
  
  // Ok now spawn a receiving thread.
  if (pthread_create(&_receive_thread,
                     NULL,
                     nic_t::staticFunction,
                     this)) {
    fprintf(stderr, "Error: Couldn't create remote memory receive thread.\n");
    return false;
  }
  
  // Init is not complete until the listener thread has signalled its readiness.
  pthread_mutex_lock(&_lock);
  while (!_listener_up) {
    pthread_cond_wait(&_listener_up_cond, &_lock);
  }
  pthread_mutex_unlock(&_lock);
  printf("Init complete.\n");
  return true;
}

void *nic_t::listen_for_protocol() {
  printf("Protocol listener up.\n");
  pthread_mutex_lock(&_lock);
  char *interface_name = _interface_name;
  struct ifreq interface_index = _interface_index;
  struct ifreq interface_mac = _interface_mac;
  pthread_mutex_unlock(&_lock);

  char buffer[RECEIVE_BUFFER_SIZE_BYTES];
  memset(buffer, 0, RECEIVE_BUFFER_SIZE_BYTES);

  _rx_socket_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
  if (_rx_socket_fd == -1) {
    perror("Listener socket");
    return NULL;
  }

  int status = 0;

  //Allowing port and addr be resusable using setsockopt
  int sockopt = 1;
  status = setsockopt(_rx_socket_fd,
                      SOL_SOCKET,
                      SO_REUSEADDR,
                      &sockopt,
                      sizeof(sockopt));
  if (status < 0) {
    perror("setsockopt");
    close(_rx_socket_fd);
    return NULL;
  }

  status = setsockopt(_rx_socket_fd,
                      SOL_SOCKET,
                      SO_REUSEPORT,
                      &sockopt,
                      sizeof(sockopt));
  if (status < 0) {
    perror("setsockopt");
    close(_rx_socket_fd);
    return NULL;
  }

  //Bind socket to iterface:eth0, interface_index is the index of interface:etho, obtained during setup()
  struct sockaddr_ll ll_addr;
  ll_addr.sll_family = PF_PACKET;
  ll_addr.sll_ifindex = interface_index.ifr_ifindex;
  ll_addr.sll_protocol = htons(ETH_P_ALL);
  status = bind(_rx_socket_fd, (struct sockaddr*)&ll_addr,
                sizeof(struct sockaddr_ll));

  if (status < 0) {
    perror("bind failed\n");
    close(_rx_socket_fd);
    return NULL;
  }

  // Now that the socket is up, but just before we block to listen, we signal
  // that we're ready.

  pthread_mutex_lock(&_lock);
  _listener_up = true;
  pthread_cond_signal(&_listener_up_cond);
  pthread_mutex_unlock(&_lock);

  while (_keep_listening) {
    // TODO(growly): This needs a timeout so that termination is graceful.
    int num_bytes = read(_rx_socket_fd, _real_recv_buffer, RECEIVE_BUFFER_SIZE_BYTES);
    
    pthread_mutex_lock(&_lock);
    if(_recv_comp_count < _recv_buffer.size()) {
      assert(_recv_buffer[_recv_comp_count].first == -1);
      _recv_buffer[_recv_comp_count].first = num_bytes;
      memcpy((void *)_recv_buffer[_recv_comp_count].second, _real_recv_buffer, num_bytes);
      _recv_comp_count++;
    }
    pthread_mutex_unlock(&_lock);
  }
  close(_rx_socket_fd);
  return NULL;
}

void* nic_t::staticFunction(void* p) {
  static_cast<nic_t*>(p)->listen_for_protocol();
  return NULL;
}

nic_t::~nic_t() {
  _keep_listening = false;
  free(_real_recv_buffer);
}
