#ifndef _GAME_DEFINITIONS_H_
#define _GAME_DEFINITIONS_H_

/** \brief Defines the type/use of a GameObject
 * Defines the type/use of a GameObject.
 */
enum GameObjectType {GO_GENERIC,
					GO_KEY,
					GO_LOCK,
					GO_DOOR,
					GO_TREASURE};

/** \brief Defines the role/use of a WayPoint
 * Defines the role/use of a WayPoint. (Not used currently).
 */
enum WayPoint_Role {WP_ROLE_NONE,
					WP_ROLE_TREASURE,
					WP_ROLE_KEY,
					WP_ROLE_DOOR};
					
/** \brief Defines the states of a Game
 * Defines the states of a Game.
 */
enum GameState {	GS_START = 0,
					GS_KEY_1 = 1,
					GS_KEY_2 = 2,
					GS_KEY_3 = 3,
					GS_KEY_4 = 4,
					GS_DOOR_OPEN = 5,
					GS_FOUND_TREASURE = 6 };

#endif
