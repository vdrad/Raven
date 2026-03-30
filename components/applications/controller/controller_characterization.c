// WORK IN PROGRESS

// // 1. Calcula o Feedforward baseado na velocidade ALVO (Setpoint)
// float feedforward = (Kv * right_motor_pid.setpoint) + V_deadband;

// // 2. Calcula o PID baseado no ERRO (Setpoint - Leitura)
// pid_compute(&right_motor_pid);

// // 3. Soma os dois! O FF faz o trabalho pesado, o PID faz o ajuste fino.
// float final_voltage = feedforward + right_motor_pid.output;

// motor_set_voltage(MOTOR_RIGHT, final_voltage);