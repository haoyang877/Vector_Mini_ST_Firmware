/**
  ******************************************************************************
  * @file    boot_handoff.h
  * @brief   Single source of truth for the APP<->Loader boot-handoff commands.
  *
  * These are the only APP CAN command numbers the boot flow depends on across
  * three owners (APP firmware, Loader tooling, host scripts):
  *
  *   BOOT_HANDOFF_CMD_ENTER_BOOT (0x66):
  *       host -> APP: safe-stop the motor, latch the boot mailbox, reset
  *   BOOT_HANDOFF_CMD_GET_MODE (0x01):
  *       host -> APP: read MotorControl.ModeNow; also used as the
  *       "APP alive again" poll after an upgrade
  *
  * The APP enum (Communication/interface_can.h) and loader/tools/loader_proto.py
  * must reference these macros, never literals: tests/test_protocol_single_source.py
  * fails when a copy reappears.  The mailbox format itself is defined in
  * shared/boot_mailbox.h.
  ******************************************************************************
  */
#ifndef BOOT_HANDOFF_H
#define BOOT_HANDOFF_H

#define BOOT_HANDOFF_CMD_ENTER_BOOT   0x66U
#define BOOT_HANDOFF_CMD_GET_MODE     0x01U

#endif /* BOOT_HANDOFF_H */
