# Assignment 2: Token Ring Simulation
**Author: Soohwan Kim (1349765)**

## **Assignment Description**
This program simulates a Token Ring Network 
using Inter-Process Communication IPC mechanisms 
such as shared memory and semaphores. 
The simulation is designed to model the behavior of a token-passing network 
where multiple processes nodes communicate in a circular fashion, 
passing a token that grants permission to send data.

The goal of this project is to provide a realistic simulation of how data packets are transferred
 in a token ring network, ensuring proper synchronization between processes and preventing deadlocks.

## **Program Design and Architecture**

### **1. System Components**
The simulation consists of the following key components:

1. **Parent Process (Main Controller)**
   - Initializes shared memory and semaphores.
   - Forks child processes for each node.
   - Randomly generates data packets and assigns them to nodes.
   - Waits for all packets to be processed and then terminates the simulation.

2. **Child Processes (Token Ring Nodes)**
   - Each child process acts as a node in the token ring.
   - Continuously listens for incoming data.
   - Forwards packets if the destination is not the current node.
   - If the node is the destination, it receives the data.
   - If the node has data to send and holds the token, it transmits a packet.

3. **Shared Memory (Inter-Process Communication)**
   - Stores the packet transfer state and semaphore-controlled synchronization variables.
   - Includes a structure that holds each node's `sent` and `received` counters.
   
4. **Semaphores (Synchronization and Mutual Exclusion)**
   - Used to coordinate access to shared data between processes.
   - Ensures that only one process accesses critical sections at a time.
   - Controls when a process can send data and when it must wait.

### **2. Packet Transmission Workflow**
1. The parent process generates a packet with a random source and destination node.
2. The token starts at **Node 0** and circulates through the network.
3. A node that receives the token:
   - If it has data to send, it transmits the packet and sends the token at the end with the packet.
   - If it does not have data, it forwards the token to the next node.
4. When the packet reaches its destination:
   - The receiving node reads and stores data by doing received++ and passes data along.
   - The token continues circulating.
5. The process repeats until all packets have been delivered.
6. The parent process waits for child nodes to finish processing, then cleans up and terminates.

### **3. Process Synchronization using Semaphores**
To prevent data corruption and ensure orderly execution, several semaphores are used:
- **TO_SEND(n)**: Controls when a node can insert a new packet.
- **EMPTY(n)** & **FILLED(n)**: Used for coordinating byte-by-byte transfer between adjacent nodes.
- **CRIT**: Ensures mutual exclusion while modifying shared data.

### **4. Simulation Execution**
1. The program is run as follows:
   - ./tokensim <number_of_packets>
   
2. After execution, the program prints statistics for each node:
	A packet is being sent : from=1, to=3, length=83
	A packet is being sent : from=3, to=0, length=45
	A packet is being sent : from=5, to=6, length=240
	...
   	Node 0: sent=144 received=154
   	Node 1: sent=0 received=395
   	Node 2: sent=0 received=26
   	...


### **5. Debugging and Packet Tracking**
To enable debugging and visualize packet movement in the network, compile the program with -DDEBUG :

CFLAGS		= -pedantic -Wall -DDEBUG

This will enable additional debug messages showing:
- When a node receives the token
- When a packet is being sent and received
- Semaphore synchronization logs

## **Summary of Results**
The simulation successfully models a **Token Ring Network**, ensuring that:
- Packets are delivered correctly to their intended destinations.
- Nodes do not send data unless they hold the token.
- Synchronization prevents race conditions and deadlocks.
- Statistics correctly reflect packet transmission.