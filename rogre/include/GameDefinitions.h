#ifndef _GAME_DEFINITIONS_H_
#define _GAME_DEFINITIONS_H_

enum WayPoint_Role {WP_ROLE_NONE,
					WP_ROLE_TREASURE,
					WP_ROLE_KEY,
					WP_ROLE_DOOR};
					
enum GameState {	GS_START,
					GS_KEY_1,
					GS_KEY_2,
					GS_KEY_3,
					GS_KEY_4,
					GS_DOOR_OPEN,
					GS_FOUND_TREASURE  };

#endif
