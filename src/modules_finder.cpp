#include "babbling_module/modules_finder.h"

ModulesFinder::ModulesFinder(ros::NodeHandle *nh) : _nh(nh), _port(0), _dist_port(0), _thread_loop_condition(false), _socket_reader_thread(&ModulesFinder::_process, this)
{

}

void ModulesFinder::setInterface(const std::string &iface)
{
    _iface_name = iface;
}

void ModulesFinder::setLocalPort(const int &port)
{
    _port = port;
}

void ModulesFinder::setDistantPort(const int &port)
{
    _dist_port = port;
}

void ModulesFinder::open()
{
    _socket_mutex.lock();

    _sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    int broadcastEnable = 1;
    setsockopt(_sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    struct ifaddrs *ifap;
    struct ifaddrs *ifac;

    if (getifaddrs(&ifap) < 0) { // Get the linked list of all network interfaces on this computer
        std::cerr << strerror(errno) << std::endl;
        _socket_mutex.unlock();
        return;
    }

    for (ifac = ifap; ifac != NULL; ifac = ifac->ifa_next) { // Iterate over the list to find the one corresponding to _iface_name
        if (_iface_name.compare(ifac->ifa_name) == 0 && ifac->ifa_addr->sa_family == AF_INET) {
            _local_sa = reinterpret_cast<struct sockaddr_in *>(ifac->ifa_addr);
            _local_netmask = reinterpret_cast<struct sockaddr_in *>(ifac->ifa_netmask);
            _local_sa->sin_port = htons(_port);

            if (bind(_sockfd, ifac->ifa_addr, sizeof(*(ifac->ifa_addr))) < 0) { // When found, bind the socket to it
                std::cerr << strerror(errno) << std::endl;
                continue;
            }
        }
    }
    freeifaddrs(ifap);

    _socket_mutex.unlock();
}

void ModulesFinder::close()
{
    _modules.clear();

    ::close(_sockfd);
}

void ModulesFinder::scan()
{
    char msg[8] = {0};
    msg[0] = BAB_CMD_WHOAREYOU;

    struct sockaddr_in dest;
    dest.sin_family = AF_INET;
    dest.sin_port = htons(_dist_port);
    dest.sin_addr.s_addr = _local_sa->sin_addr.s_addr | ~(_local_netmask->sin_addr.s_addr); // This is the interface's broadcast address

    _socket_mutex.lock();
    if (sendto(_sockfd, msg, 1, 0, reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest)) < 0) { // Send the WHOAREYOU command to everything connected to the network
        std::cerr << strerror(errno) << std::endl;
        _socket_mutex.unlock();
        return;
    }
    _socket_mutex.unlock();
}

void ModulesFinder::_process()
{
    while (!_thread_loop_condition) ; // Wait for start to be called

    std::cout << "Now listening on " << _iface_name << "." << std::endl;
    
    struct sockaddr sa;
    socklen_t src_sz = sizeof(sa);
    char msg[32];
    ssize_t r_sz;

    do {
        std::for_each(_modules.begin(), _modules.end(), [&](std::pair<uint32_t, Module *> p) { // If a module timed out, remove it from the list
            if (!std::get<1>(p)->isOnline()) {
                delete std::get<1>(p);
                _modules.erase(std::get<0>(p));
            }
        });

        _socket_mutex.lock();
        r_sz = recvfrom(_sockfd, &msg, 32, MSG_DONTWAIT, &sa, &src_sz); // Grab a packet from the socket
        _socket_mutex.unlock();

        if (r_sz < 0) { // No packet was available
            continue;
        }

        uint32_t src_ip = reinterpret_cast<struct sockaddr_in *>(&sa)->sin_addr.s_addr;

        if (_modules.find(src_ip) != _modules.end()) { // If the source ip is known, dispatch the packet to the corresponding module
            _modules[src_ip]->process(msg, r_sz);
        } else if (r_sz > 1 && msg[0] == BAB_CMD_WHOIAM) {
            std::cout << "Received from " << std::hex << src_ip << std::endl;
            std::cout << "  Type : " << id_to_type[msg[1]] << std::endl;
            if (r_sz > 2 && msg[1] == BAB_TYPE_MODULE) {
                std::cout << "    Module type : " << id_to_module_type[msg[2]] << std::endl;
                Module *mod_pr = ModuleFactory::fromID(msg[2], reinterpret_cast<uint8_t *>(&msg[3]), sa, _sockfd, _nh); // If not, and if the message starts with an identification, create a new Module
                if (mod_pr != NULL) {
                    mod_pr->start();
                    _modules.insert(std::pair<uint32_t, Module *>(src_ip, mod_pr));
                }
            }
        }
    } while (_thread_loop_condition);
}

void ModulesFinder::start()
{
    _thread_loop_condition = true;
}

void ModulesFinder::stop()
{
    std::for_each(_modules.begin(), _modules.end(), [](std::pair<uint32_t, Module *> p) { // Stop all modules
        std::get<1>(p)->stop();
    });

    _thread_loop_condition = false;
    _socket_reader_thread.join();
}
