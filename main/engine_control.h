
#ifndef ENGINE_CONTROL_H
#define ENGINE_CONTROL_H

/* INCLUDES *******************************************************************/
/******************************************************************************/

/* DEFINES ********************************************************************/
/******************************************************************************/

/* ENUMS **********************************************************************/
typedef enum {
    RUNNING,
    OFF
} engine_state_e;
/******************************************************************************/

/* STRUCTURES *****************************************************************/
/******************************************************************************/

/* GLOBALS ********************************************************************/
/******************************************************************************/

/* PROTOTYPES *****************************************************************/
void engine_control_init(void);
void engine_control_toggle_engine_state(void);
void engine_control_start_engine(void);
void engine_control_stop_engine(void);
/******************************************************************************/

#endif /* #ifndef ENGINE_CONTROL_H */