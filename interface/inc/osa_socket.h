#ifndef OSA_SOCKET_H
#define OSA_SOCKET_H

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>


int OSA_Set_keepalive(int sd,int keepalive,int keepinterval,int keepidle,int keepcount);

int OSA_Set_Nodelay(int sd);

int OSA_create_server_socket(int server_port,int backlog);

int OSA_create_server_socket_udp(int server_port);

int OSA_server_accept(int server_sd,struct sockaddr *addr);

int OSA_socket_recv_data(int sd,void *buf,unsigned int len);

int OSA_socket_send_data(int sd,void *buf,unsigned int len);

int OSA_socket_get_tcpconnect_state(int sd);

int OSA_delete_server_socket(int sd);

int OSA_udp_recv_data(int sd,char *buf,unsigned int size);

int OSA_udp_send_data(int sd,char *ip,unsigned int port,char *buf,unsigned int size);
#endif
