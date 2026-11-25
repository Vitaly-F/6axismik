/*
 * Разбор команд от компьютера. Одна команда = одна строка, конец строки '\n'.
 * На каждую строку плата отвечает ровно одной строкой "ok" или "error: ...".
 * Описание команд — в README.md и по команде H.
 */
#ifndef COMMANDS_H
#define COMMANDS_H

void commands_init(void);
void commands_poll(void); /* вызывать в основном цикле */

#endif
