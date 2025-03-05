/*
 * The program simulates a Token Ring LAN by forking off a process
 * for each LAN node, that communicate via shared memory, instead
 * of network cables. To keep the implementation simple, it jiggles
 * out bytes instead of bits.
 *
 * It keeps a count of packets sent and received for each node.
 */
#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "tokenRing.h"


/*
 * This function is the body of a child process emulating a node.
 */
void
token_node(control, num)
	struct TokenRingData *control;
	int num;
{
	int rcv_state = TOKEN_FLAG, rcv_to = 0, rcv_from = 0, not_done = 1, data_count = 0, len;
	unsigned char byte;


	/*
	 * If this is node #0, start the ball rolling by creating the
	 * token.
	 */
	if(num == 0){
		send_byte(control, 0, 0x01);
	}

	/*
	 * Loop around processing data, until done.
	 */
	while (not_done) {
		byte = rcv_byte(control, num);
		if(byte == -9999){
			//previous node got terminated
#ifdef DEBUG
			fprintf(stderr, "[TOKEN_NODE] Node %d sees terminate=1, will exit\n", num);
#endif
			not_done = 0;
			break;
		}

		/*
		 * Handle the byte, based upon current state.
		 */
		switch (rcv_state) {
		case TOKEN_FLAG:
			if(byte == 0x01){
				// If we have data to send, send it.
                if (control->shared_ptr->node[num].to_send.length > 0) {
#ifdef DEBUG
                    fprintf(stderr, "[TOKEN_NODE] Node %d has a packet of length=%d to send\n",
                            num, control->shared_ptr->node[num].to_send.length);
#endif
					control->shared_ptr->node[num].to_send.token_flag = TOKEN_FLAG;
					while(1){
						if (control->shared_ptr->node[num].terminate == 1) {
#ifdef DEBUG
							fprintf(stderr, "[TOKEN_NODE] Node %d sees terminate=1, will exit\n", num);
#endif
							send_byte(control, num, -9999);
							not_done = 0;
							break;
						}
						send_pkt(control, num);
						if(control->snd_state == TOKEN_FLAG) break;
					}
                } else { // Otherwise, pass the token along.
#ifdef DEBUG
                    fprintf(stderr, "[TOKEN_NODE] Node %d passes token along\n", num);
#endif
					if (control->shared_ptr->node[num].terminate == 1) {
#ifdef DEBUG
						fprintf(stderr, "[TOKEN_NODE] Node %d sees terminate=1, will exit\n", num);
#endif
						send_byte(control, num, -9999);
						not_done = 0;
						break;
					}
                    send_byte(control, num, 0x01);
                }
			} else { // If it's a data byte, go to TO state.
				WAIT_SEM(control, CRIT);
				control->shared_ptr->node[num].to_send.token_flag = 0x00;
				SIGNAL_SEM(control, CRIT);
				rcv_state = TO;
			}
			break;

		case TO:
			WAIT_SEM(control, CRIT);
			control->shared_ptr->node[num].to_send.to = byte;
			SIGNAL_SEM(control, CRIT);
			rcv_to = byte;
			rcv_state = FROM;
			break;

		case FROM:
			WAIT_SEM(control, CRIT);
			control->shared_ptr->node[num].to_send.from = byte;
			SIGNAL_SEM(control, CRIT);
			rcv_from = byte;
			rcv_state = LEN;
			break;

		case LEN:
			WAIT_SEM(control, CRIT);
			control->shared_ptr->node[num].to_send.length = byte;
			SIGNAL_SEM(control, CRIT);
			len = byte;
			rcv_state = DATA;
			data_count = 0;
			break;

		case DATA:
			//data getting in debug message
#ifdef DEBUG
			fprintf(stderr, "[TOKEN_NODE] Node %d received data byte=0x%02X, count=%d/%d\n",
					num, byte, data_count, len);
#endif

			// If the data is from this node, discard the data
			if(rcv_from == num){
                if (data_count == len - 1) {
					// discard data if it's from this node
#ifdef DEBUG
					fprintf(stderr, "[TOKEN_NODE] Node %d discarding data\n", num);
#endif

					WAIT_SEM(control, CRIT);
					control->shared_ptr->node[num].sent++;
					control->shared_ptr->node[num].to_send.token_flag = TOKEN_FLAG;
					control->shared_ptr->node[num].to_send.to = 0;
					control->shared_ptr->node[num].to_send.from = 0;
					control->shared_ptr->node[num].to_send.length = 0;
					memset(control->shared_ptr->node[num].to_send.data, 0, MAX_DATA);
					SIGNAL_SEM(control, CRIT);
					rcv_state = TOKEN_FLAG;
                    data_count = 0;
					kill(getppid(), SIGUSR1);
					SIGNAL_SEM(control, TO_SEND(num));

                }
			}else if(rcv_to == num){ // if the data is for this node, receive it and pass data and token along
				WAIT_SEM(control, CRIT);
				control->shared_ptr->node[num].to_send.data[data_count] = byte;
				SIGNAL_SEM(control, CRIT);
				if (data_count == len - 1) {
					control->shared_ptr->node[num].received++;
					rcv_state = TOKEN_FLAG;
                    data_count = 0;
					//recieved completed
#ifdef DEBUG
					fprintf(stderr, "[TOKEN_NODE] Node %d received data completed\n", num);
#endif
                }
			}else {  // Otherwise, forward the data
				// forward the data
				WAIT_SEM(control, CRIT);
				control->shared_ptr->node[num].to_send.data[data_count] = byte;
				SIGNAL_SEM(control, CRIT);
				if (data_count == len - 1) {
					rcv_state = TOKEN_FLAG;
                    data_count = 0;
                }
			}

			data_count++;
			break;
		};

		if (control->shared_ptr->node[num].terminate == 1) {
#ifdef DEBUG
            fprintf(stderr, "[TOKEN_NODE] Node %d sees terminate=1, will exit\n", num);
#endif
			send_byte(control, num, -9999);
            not_done = 0;
			break;
        }
	}
}

/*
 * This function sends a data packet followed by the token, one byte each
 * time it is called.
 */
void
send_pkt(control, num)
	struct TokenRingData *control;
	int num;
{
	struct data_pkt *p = &control->shared_ptr->node[num].to_send;
	static int sndpos, sndlen;
	sndlen = p->length;


	switch (control->snd_state) {
	case TOKEN_FLAG:
		send_byte(control, num, 0x00);
		control->snd_state = TO;
		break;

	case TO:
		send_byte(control, num, (unsigned char)p->to);
		control->snd_state = FROM;
		break;

	case FROM:
		send_byte(control, num, (unsigned char)p->from);
		control->snd_state = LEN;
		break;

	case LEN:
		send_byte(control, num, (unsigned char)p->length);
		sndpos = 0;
		control->snd_state = DATA;
		break;

	case DATA:
#ifdef DEBUG
		fprintf(stderr, "[SEND_PKT] Node %d: state=DATA, sending data[%d]=0x%02X\n",
			num, sndpos, (unsigned char)p->data[sndpos]);
#endif
		send_byte(control, num, (unsigned char)p->data[sndpos]);
		(sndpos)++;

		if (sndpos >= sndlen) {
#ifdef DEBUG
			fprintf(stderr, "[SEND_PKT] Node %d: finished packet, send token + signal TO_SEND\n", num);
#endif
			WAIT_SEM(control, CRIT);
			control->shared_ptr->node[num].to_send.token_flag = TOKEN_FLAG;
			control->shared_ptr->node[num].to_send.to = 0;
			control->shared_ptr->node[num].to_send.from = 0;
			control->shared_ptr->node[num].to_send.length = 0;
			memset(control->shared_ptr->node[num].to_send.data, 0, MAX_DATA);
			SIGNAL_SEM(control, CRIT);

			sndpos = 0;
			sndlen = 0;
			send_byte(control, num, 0x01); // token sending
	

			control->snd_state = DONE;
		}
		break;

		case DONE:
#ifdef DEBUG
			fprintf(stderr, "[SEND_PKT] Node %d: state=DONE, no more bytes to send\n", num);
#endif
			control->snd_state = TOKEN_FLAG;
			break;
	};
}

/*
 * Send a byte to the next node on the ring.
 */
void
send_byte(control, num, byte)
	struct TokenRingData *control;
	int num;
	unsigned byte;
{
    WAIT_SEM(control, EMPTY(num));
    control->shared_ptr->node[num].data_xfer = byte;   
    SIGNAL_SEM(control, FILLED(num));
}

/*
 * Receive a byte for this node.
 */
unsigned char
rcv_byte(control, num)
	struct TokenRingData *control;
	int num;
{
    unsigned char byte;
    int prev = (num + N_NODES - 1) % N_NODES;

    WAIT_SEM(control, FILLED(prev));

    byte = control->shared_ptr->node[prev].data_xfer;


    SIGNAL_SEM(control, EMPTY(prev));


    return byte;
}

