#include "telemetry_server.h"

/**
 * update_json_pkg - constantly listens for incoming json from uavs
 */
void update_json_pkg() {
	//listen on port
	//update as necessary
}

/**
 * json_to_rust - sends the json_pkg to the rust server
 * Return: 1 if successful, 0 if not
 */
int json_to_rust() {
	//open stream to Rust
	//send
	//await reply
	//report 0 if unsuccessful

	return (1);
}

/**
 * jston_from_rust - receives the json_pkg from the rust server
 */
int json_from_rust() {
	return (1);
}