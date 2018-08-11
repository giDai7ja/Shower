
.device	ATtiny2313A
.include "tn2313Adef.inc"

.def temp = r16						; рабочая переменная
.def data = r17						; байт для приёма/передачи
.def count = r15					; счётчик для циклов
.def count2 = r14					; еще счётчик

.def Razr0 = r22 ;разряды задержки
.def Razr1 = r23
.def Razr2 = r21

.set		PIN_BUZ		= PINB3
.set		PORT_BUZ	= PORTB

.set		PIN_ECHO	= PIND6
.set		PORT_ECHO	= PORTD

.set		PIN_TRIG	= PINB4
.set		PORT_TRIG	= PORTB	

; EEPROM
.eseg
			MIN_UR:		.dw	800			; Калибровочное значение
			MAX_UR:		.dw	8000		; Калибровочное значение
			TTT:		.dw 0
			YYY:		.dw	0

.dseg
.org	SRAM_START

ds1820:		.byte	8
tm:			.byte	2
LED_N:		.byte	1
LED_BUF:	.byte	5
FLAG:		.byte	1					; 0 - Конец измерения уровня
										; 1 - Переполнение счётчика уровня
										; 2 - Кнопка "Не пищать" была нажата
										; 3 - Происходит конвертация температуры
										; 4 - Нажата калибровка верхнего уровня
										; 5 - Нажата калибровка нижнего уровня
										; 6 - 
										; 7 - 
COUNT_T1:	.byte	2
COUNT_T2:	.byte	2
MIN_U:		.byte	2
MAX_U:		.byte	2
RATIO:		.byte	1


.cseg

.org 0
									; Таблица векторов прерываний
			rjmp	RESET			; External Pin, Power-on Reset, Brown-out Reset, and Watchdog Reset
			rjmp	INT0_INT		;reti					; INT0 External Interrupt Request 0
			reti					; INT1 External Interrupt Request 1
			rjmp	T1_CAPT			; TIMER1 CAPT Timer/Counter1 Capture Event
			reti					; TIMER1 COMPA Timer/Counter1 Compare Match A
			rjmp	TIMER1_OVF		; TIMER1 OVF Timer/Counter1 Overflow
			reti	;rjmp	TIMER0_OVF		; TIMER0 OVF Timer/Counter0 Overflow
			reti					; USART0, RX  USART0, Rx Complete
			reti					; USART0, UDRE USART0 Data Register Empty
			reti					; USART0, TX USART0, Tx Complete
			reti					; ANALOG COMP Analog Comparator
			reti					; PCINT0 Pin Change Interrupt Request 0
			reti					; TIMER1 COMPB Timer/Counter1 Compare Match B
			rjmp	TIMER0_COMPA	; TIMER0 COMPA Timer/Counter0 Compare Match A
			reti					; TIMER0 COMPB Timer/Counter0 Compare Match B
			reti					; USI START USI Start Condition
			reti					; USI OVERFLOW USI Overflow
			reti					; EE READY EEPROM Ready
			reti					; WDT OVERFLOW Watchdog Timer Overflow
			reti					; PCINT1 Pin Change Interrupt Request 1
			rjmp	PCINT2_INT		; PCINT2 Pin Change Interrupt Request 2

.org	INT_VECTORS_SIZE

CHARS:		.db		0b00111111,\
					0b00000110,\
					0b01011011,\
					0b01001111,\
					0b01100110,\
					0b01101101,\
					0b01111101,\
					0b00000111,\
					0b01111111,\
					0b01101111,\
					0b01110111,\
					0b01111100,\
					0b00111001,\
					0b01011110,\
					0b01111001,\
					0b01110001


OW_INIT:
			ldi		temp,(1<<RXEN|1<<TXEN|1<<RXB8|1<<TXB8)
			out		UCSRB,temp		;разрешение приема/передачи 8 бит
			ldi		temp,(1<<U2X)
			out		UCSRA,temp
;			rcall	OW_115200
			ret

OW_9600:							; Инициализация USART на скорости 9600
			ldi		temp,155		;скорость передачи 9600 при 8 МГц (12/103)
			out		UBRR,temp		;устанавливаем скорость
			ret

OW_115200:							; Инициализация USART на скорости 115200
			ldi		temp,12			;скорость передачи 115200 при 8 МГц (0/8)
			out		UBRR,temp		;устанавливаем скорость
			ret

OW_SEND:							;посылка байта из temp с ожиданием готовности
			sbis	UCSRA,UDRE		;для Classic UCSRA заменить на USR
			rjmp	OW_SEND
			out		UDR,temp		;собственно посылка байта
In_com:								;прием байта в temp с ожиданием готовности
			sbis	UCSRA,RXC		;для Classic UCSRA заменить на USR
			rjmp	In_com
			in		temp,UDR		;собственно прием байта

			ret

OW_SEND_BYTE:						; Байт для передачи в data
			ldi		temp,8			; число бит в байте
			mov		count,temp
TXNB:		
			sbrs	data,0			; проверяем бит 0
			rjmp	TX0			; если 0 - передать бит 0
			ldi		temp,0xFF
			rcall	OW_SEND			; посылаем бит 1
			rjmp	TX_NEXT		; продолжить
TX0:		ldi		temp,0x00
			rcall	OW_SEND			; посылаем бит 0

TX_NEXT:	lsr		data			; сдвигаем данные вправо
			dec		count			; уменьшаем счётчик
			brne	TXNB			; повторяем для следующего бита

			ret						; выходим, если битов нет

OW_RECEIVE_BYTE:
			ldi		temp,8			; число бит в байте
			mov		count,temp

			clr		data
RXNB:
			ldi		temp,0xFF		; Посылаем 0xFF
			rcall	OW_SEND
			cpi		temp,0xFF		; Если принято 0xFF - приняли единичку
			breq	BIT_1		
			clc
			rjmp	BIT_0			; Иначе - приняли ноль
BIT_1:		sec
BIT_0:		ror		data			; сдвигаем

			dec		count
			brne	RXNB				; принять следующий бит

			ret

OW_RESET:
			rcall	OW_9600			; Сигнал сброс + присутствие

			ldi		temp,0xF0
			rcall	OW_SEND
			mov		r1,temp
			rcall	OW_115200
			ret


;---------------------------------------------------;
;                     Прерывание INT0               ;
;---------------------------------------------------;

INT0_INT:
			push	r16

			lds		r16,FLAG
			sbr		r16,0b00000100
			sts		FLAG,r16

			pop		r16
			reti

;---------------------------------------------------;
;         Прерывание динамической индикации         ;
;---------------------------------------------------;

TIMER0_COMPA:
			push		r19
			push		r20
			push		r21
			in			r19,SREG
			push		r19
			mov			r19,YL
			push		r19
			mov			r19,YH
			push		r19
			
			lds		r19,LED_N

			cpi		r19,4
			breq	TL1
TL0:		inc		r19
			rjmp	TL2
TL1:		clr		r19
TL2:		sts		LED_N,r19

			cbi		PORTB,PINB0		; Ноль на SS

			ldi		r20,4
TL3:		cp		r19,r20
			breq	TL4
			cbi		PORTB,PINB1
			rjmp	TL5
TL4:		sbi		PORTB,PINB1			

TL5:		sbi		PORTB,PINB2		; Дергаем SCK
			cbi		PORTB,PINB2

			tst		r20
			breq	TL6
			dec		r20
			rjmp	TL3


TL6:		
			ldi		YH,high(LED_BUF)
			ldi		YL,low(LED_BUF)

			add		YL,r19

			ld		r20,Y

			ldi		r21,8
			
TL7:		rol		r20
			brcs	TL8
			cbi		PORTB,PINB1
			rjmp	TL9

TL8:		sbi		PORTB,PINB1

TL9:		sbi		PORTB,PINB2		; Дергаем SCK
			cbi		PORTB,PINB2

			dec		r21
			brne	TL7


			sbi		PORTB,PINB0		; Единичка на SS

			tst		r5
			breq	SCHED
			dec		r5

			tst		r8
			brne	BUZZ_ON
			cbi		PORTB,PINB3		
			rjmp	SCHED
BUZZ_ON:
			sbi		PORTB,PINB3
			dec		r8
			
SCHED:

			pop		r19
			mov		YH,r19
			pop		r19
			mov		YL,r19
			pop		r19
			out		SREG,r19
			pop		r21
			pop		r20
			pop		r19

			reti



;-------------------------------------------------------------;
;			 Прерывание захвата таймера T1                    ;
;-------------------------------------------------------------;

T1_CAPT:
			push	r19
			in		r19,SREG
			push	r19

			in		r19,TCCR1B
			sbrs	r19,ICES1
			rjmp	T1_READY

			cbr		r19,(1<<ICES1)
			out		TCCR1B,r19

			in		r19,ICR1L
			sts		COUNT_T1,r19
			in		r19,ICR1H
			sts		(COUNT_T1+1),r19

			rjmp	T1_END_I

T1_READY:
			clr		r19							; останавливаем счётчик
			out		TCCR1B,r19

			in		r19,TIMSK
			cbr		r19,(1<<ICIE1)
			out		TIMSK,r19

			in		r19,ICR1L
			sts		COUNT_T2,r19
			in		r19,ICR1H
			sts		(COUNT_T2+1),r19

			lds		r19,FLAG
			sbr		r19,0x01
			sts		FLAG,r19

T1_END_I:
			pop		r19
			out		SREG,r19
			pop		r19

			reti



;---------------------------------------------------------------;
;               Прерывание переполнение таймера 1               ;
;---------------------------------------------------------------;

TIMER1_OVF:
			push	r19
			;in		r19,SREG
			;push	r19

			clr		r19						; останавливаем счётчик
			out		TCCR1B,r19

			lds		r19,FLAG
			sbr		r19,0x03
			sts		FLAG,r19

			;pop		r19
			;out		SREG,r19
			pop		r19

			reti

;------------------------------------------------------------------;
;           Прерывание от кнопок калибровки                        ;
;------------------------------------------------------------------;

PCINT2_INT:
			push	r16
			in		r16,SREG
			push	r16

			in		r16,GIMSK			; Запрещаем прерывания от кнопок
			cbr		temp,(1<<PCIE2)		; Устраним дребезг контактов
			out		GIMSK,temp

			lds		r16,FLAG

			sbis	PIND,PIND4
			sbr		r16,0b00010000

			sbis	PIND,PIND5
			sbr		r16,0b00100000

			sts		FLAG,r16

			andi	r16,0b00110000
			breq	PCINT2_END
			ldi		r16,75
			mov		r8,r16

PCINT2_END:
			pop		r16
			out		SREG,r16
			pop		r16

			reti

;------------------------------------------------------------------;
;           Начало основной программы                              ;
;------------------------------------------------------------------;

RESET:
			cli
			ldi		temp,RAMEND		; Инициализация стека
			out		SPL,temp

			ldi		YH,high(LED_BUF)
			ldi		YL,low(LED_BUF)

			ldi		r21,5

res01:
			ldi		temp,0b11111111
			st		Y+,temp
			dec		r21
			brne	res01
			
			clr		temp			; Инициализация переменной для динамической индикации
			sts		LED_N,temp
			mov		r8,temp
			ldi		temp,0b00000100
			sts		FLAG,temp
			;out		TCNT1H,r16
			;out		TCNT1L,r16

			ldi		temp,15
			mov		r5,temp
			mov		r6,temp
			mov		r7,temp
									
			ldi		temp,(1<<ACD)
			out		ACSR,temp

			ldi		temp,(1<<PRUSI)
			out		PRR,temp

			ldi		temp,(1<<PINB0)|(1<<PINB1)|(1<<PINB2)|(1<<PINB3)|(1<<PIN_TRIG)
			out		DDRB,temp		; Порт на вывод

			cbi		PORTB,PINB3		; Не пищать

			sbi		PORTB,PINB0		; Высокий уровень на SS
			cbi		PORTB,PINB2		; Низкий уровень на SCK

			cbi		PORTB,PIN_TRIG	; Низкий уровень на TRIG

			sbi		PORTD,PIND6		; Подтяжка на ECHO

			sbi		PORTD,PIND4		; Подтяжка к питанию кнопок калибровки
			sbi		PORTD,PIND5

			ldi		temp,(1<<WGM01)	; Режим таймера CTC
			out		TCCR0A,temp

			ldi		temp,(1<<CS02)	; настраиваем предделитель /256
			out		TCCR0B,temp

			ldi		temp,(1<<TOIE1)|(1<<OCIE0A)
			out		TIMSK,temp		; Разрешаем прерывания таймера

			ldi		temp,92
			out		OCR0A,temp		; Задаём частоту для динамической индикации

			;ldi		temp,(1<<PCIE2)					; Настройка прерываний кнопок колибровки
			;out		GIMSK,temp
			ldi		temp,(1<<PCINT15)|(1<<PCINT16)
			out		PCMSK2,temp

			ldi		temp,(1<<INT0)
			out		GIMSK,temp

			ldi		temp,(1<<ISC01)
			out		MCUCR,temp

			wdr							; Инициализируем WD таймер
			ldi		temp,(1<<WDCE)|(1<<WDE)|(1<<WDP3)
			out		WDTCR,temp

			sei

			rcall	UR_INIT

			rcall	OW_INIT


;------------------------------------------------------------------;
;                Основной цикл                                     ;
;------------------------------------------------------------------;

MAIN:
			wdr

			tst		r5
			brne	MAIN_NEXT

			ldi		r16,50
			mov		r5,r16

			tst		r6
			brne	MAIN_1
			ldi		r16,10
			mov		r6,r16
			rcall	UR

MAIN_1:		
			dec		r6
				
			tst		r7
			brne	MAIN_2
			ldi		r16,88
			mov		r7,r16
			rcall	Match33h

MAIN_2:
			dec		r7

MAIN_NEXT:
			lds		r16,FLAG

			sbrc	r16,4
			rjmp	NEW_MIN

			sbrc	r16,5
			rjmp	NEW_MAX

			sbrc	r16,3
			rcall	T_READY

			lds		r16,FLAG
			sbrc	r16,0
			rcall	UR_RD

			rjmp	MAIN_END

NEW_MIN:
			ldi		r17,MIN_UR
			lds		r16,COUNT_T1
			rcall	EEWrite

			inc		r17
			lds		r16,COUNT_T1+1
			rcall	EEWrite

			rjmp	NEW_END

NEW_MAX:
			ldi		r17,MAX_UR
			lds		r16,COUNT_T1
			rcall	EEWrite

			inc		r17
			lds		r16,COUNT_T1+1
			rcall	EEWrite

NEW_END:
			lds		r16,FLAG
			cbr		r16,0b00110000
			sts		FLAG,r16

			ldi		r16,10
			mov		r6,r16
			mov		r7,r16

			rcall	UR_INIT

MAIN_END:

/*L4:*/
/*			lds		r16,FLAG
			sbrs	r16,3
			rjmp	L44_END

			ldi		temp,0xFF
			rcall	OW_SEND			; посылаем бит 1
			cpi		temp,0xFF		; Если принято 0xFF - приняли единичку
			breq	L44_END

			lds		r16,FLAG
			cbr		r16,0b00001000
			sts		FLAG,r16
			
			rcall	T_READY
L44_END:*/

			rjmp	MAIN

; Read ROM 33h
Match33h:	
			lds		r16,FLAG
			cbr		r16,0b00001000
			sts		FLAG,r16

			rcall	OW_RESET

			mov		temp,r1
			cpi		temp,0xF0		; проверка на присутствие датчика
			brne	L5

			ldi		YL,low(LED_BUF+2)
			ldi		YH,high(LED_BUF+2)

			ldi		temp,0b01000000
			st		Y+,temp
			st		Y+,temp
			st		Y,temp
			
			ret

L5:			ldi		data,0x33		; считываем 8 байт кода датчика
			rcall	OW_SEND_BYTE

			ldi		temp,8
			mov		count2,temp

			ldi		YL,Low(ds1820)
			ldi		YH,High(ds1820)

L2:			
			rcall	OW_RECEIVE_BYTE
			st		Y+,data

			dec		count2
			brne	L2

; Match ROM 55h
Match55h:	rcall	OW_RESET

			ldi		data,0x55		
			rcall	OW_SEND_BYTE

			ldi		temp,8
			mov		count2,temp

			ldi		YL,Low(ds1820)
			ldi		YH,High(ds1820)

L3:			
			ld		data,Y+
			rcall	OW_SEND_BYTE

			dec		count2
			brne	L3

; Конвертация температуры (44h)
			ldi		data,0x44		
			rcall	OW_SEND_BYTE

			lds		r16,FLAG
			sbr		r16,0b00001000
			sts		FLAG,r16

			ret

/*L4:
			ldi		temp,0xFF
			rcall	OW_SEND			; посылаем бит 1
			cpi		temp,0xFF		; Если принято 0xFF - приняли единичку
			breq	L44_END
			rjmp	L4
L44_END:			
*/
; Match ROM 55h снова
T_READY:

			ldi		temp,0xFF
			rcall	OW_SEND			; посылаем бит 1
			cpi		temp,0xFF		; Если принято 0xFF - приняли единичку
			breq	L8
			ret

L8:
			lds		r16,FLAG
			cbr		r16,0b00001000
			sts		FLAG,r16
			
/*			rcall	T_READY
L44_END:*/

			rcall	OW_RESET

			ldi		data,0x55		
			rcall	OW_SEND_BYTE

			ldi		temp,8
			mov		count2,temp

			ldi		YL,Low(ds1820)
			ldi		YH,High(ds1820)

L7:			
			ld		data,Y+
			rcall	OW_SEND_BYTE

			dec		count2
			brne	L7


; READ SCRATCHPAD [BEh] (Чтение памяти)
			ldi		data,0xBE
			rcall	OW_SEND_BYTE

			ldi		temp,2
			mov		count2,temp

			ldi		YL,Low(tm)
			ldi		YH,High(tm)

L6:			
			rcall	OW_RECEIVE_BYTE
			st		Y+,data

			dec		count2
			brne	L6



;------------- Вывод на индикатор температуры

			lds		r16,tm
			lds		r17,tm+1
			swap	r17
			andi	r17,0xF0
			swap	r16
			andi	r16,0x0F
			add		r17,r16
			;sts		tm+1,r17
			lds		r16,tm
			andi	r16,0x0F

			sts		tm,r16

			ldi		ZL,low(CHARS*2)
			ldi		ZH,high(CHARS*2)

			sbrc	r17,7
			rjmp	T_MINUS

T_PLUS:
			cpi		r17,100							; Сравним температуру с 99 градусами
			brlo	T_L99							; Переход если меньше или равно

													; Тут код вывода если Т больше 99
			inc		ZL

			lpm		r18,Z
			sts		LED_BUF+4,r18

			subi	r17,100
			
			rcall	bin2bcd8

			ldi		ZL,low(CHARS*2)

			add		ZL,r16
			lpm		r18,Z
			sts		LED_BUF+3,r18

			ldi		ZL,low(CHARS*2)
			
			add		ZL,r17
			lpm		r18,Z
			sts		LED_BUF+2,r18
			
			rjmp	T_END

T_L99:
			cpi		r17,10							; Сравним температуру с 9 градусами
			brlo	T_L9

			rcall	bin2bcd8

			ldi		ZL,low(CHARS*2)

			add		ZL,r16
			lpm		r18,Z
			sts		LED_BUF+4,r18

			ldi		ZL,low(CHARS*2)
			
			add		ZL,r17
			lpm		r18,Z
			sbr		r18,0b10000000
			sts		LED_BUF+3,r18

T_0:		
			lds		dd16uH,tm
			clr		dd16uL

			ldi		dv16uL,0x99
			ldi		dv16uH,0x01

			rcall	div16u

			ldi		ZL,low(CHARS*2)
			
			add		ZL,r16
			lpm		r18,Z
			sts		LED_BUF+2,r18
			
			rjmp	T_END

T_L9:
			clr		r18
			sts		LED_BUF+4,r18
T_Lm9:
			ldi		ZL,low(CHARS*2)
			add		ZL,r17
			lpm		r18,Z
			sbr		r18,0b10000000
			sts		LED_BUF+3,r18
			
			rjmp	T_0

T_MINUS:	
			ldi		r18,0b01000000
			sts		LED_BUF+4,r18					; Вывели минус

			com		r17
			neg		r16

			sbrs	r16,4
			inc		r17

;T_H:
			andi	r16,0x0F
			sts		tm,r16

			cpi		r17,10							; Сравним температуру с 9 градусами
			brlo	T_Lm9

			rcall	bin2bcd8

			ldi		ZL,low(CHARS*2)

			add		ZL,r16
			lpm		r18,Z
			sts		LED_BUF+3,r18

			ldi		ZL,low(CHARS*2)
			
			add		ZL,r17
			lpm		r18,Z
			sts		LED_BUF+2,r18

			rjmp	T_END		

T_END:

			ret

;-----------------------------;
;     Измерение уровня        ;
;-----------------------------;

UR:
			sbi		PORTB,PIN_TRIG		; Запускаем измерение

			ldi		Razr0,24			; Задержка примерно 10мкс
			clr		Razr1
			clr		Razr2
			rcall	Delay
			cbi		PORTB,PIN_TRIG		; Сформировали импульс

			cli
			clr		r16					; Очистим счётчик
			out		TCNT1H,r16
			out		TCNT1L,r16

			ldi		temp,(1<<ICNC1)|(1<<ICES1)|(1<<CS11)
			out		TCCR1B,temp		; // настраиваем делитель T1

			ldi		temp,(1<<ICF1)
			out		TIFR,temp

			in		temp,TIMSK
			sbr		temp,(1<<ICIE1)
			out		TIMSK,temp

			sei

			ret

UR_RD:
			ldi		YL,low(LED_BUF)
			ldi		YH,high(LED_BUF)

			lds		temp,FLAG

			sbrs	temp,1				; Проверка на переполнение счётчика
			rjmp	T1_L1

			ldi		temp,0b01000000
			st		Y+,temp
			ldi		temp,0b01000000
			st		Y+,temp

			rjmp	END_T1

T1_L1:
			ldi		ZL,low(CHARS*2)
			ldi		ZH,high(CHARS*2)

			lds		r20,COUNT_T2
			lds		r21,COUNT_T2+1

			lds		r22,COUNT_T1
			lds		r23,(COUNT_T1+1)

/*			ldi		r17,TTT
			lds		r16,COUNT_T1
			rcall	EEWrite

			ldi		r17,TTT+1
			lds		r16,COUNT_T1+1
			rcall	EEWrite

			ldi		r17,YYY
			lds		r16,COUNT_T2
			rcall	EEWrite

			ldi		r17,YYY+1
			lds		r16,COUNT_T2+1
			rcall	EEWrite

rrrrr:
			rjmp	rrrrr*/


			sub		r20,r22					; В r20 r21 значение
			sbc		r21,r23

			sts		COUNT_T1,r20
			sts		COUNT_T1+1,r21

			lds		r18,MIN_U
			lds		r19,(MIN_U+1)
			
			lds		r22,MAX_U
			lds		r23,(MAX_U+1)

			cp		r18,r20					; Проверка на верхний уровень
			cpc		r19,r21
			brsh	T1_HI

			cp		r20,r22					; Проверка на нижний уровень
			cpc		r21,r23
			brsh	T1_LOW

			sub		r22,r20
			sbc		r23,r21

			mov		dd16uH,r23
			mov		dd16uL,r22

			lds		dv16uL,RATIO
			clr		dv16uH

			rcall	div16u

			mov		data,dres16uL

;-----		Пищать тут
			
			lds		r19,FLAG

			cpi		data,95
			brlo	NO_BEEP

			sbrc	r19,2
			rjmp	NO_BEEP

			ldi		r16,50
			mov		r8,r16

NO_BEEP:
			cpi		data,90
			brsh	NO_BEEP1
			
			cbr		r19,0b00000100
			sts		FLAG,r19

NO_BEEP1:
			
;-----		Конец пищать тут

			rcall	bin2bcd8
			
			add		ZL,data

			lpm		r19,Z
			st		Y+,r19

			ldi		ZL,low(CHARS*2)
			add		ZL,temp

			lpm		r19,Z
			st		Y+,r19
			rjmp	END_T1

T1_HI:
			ldi		temp,0b00000111
			st		Y+,temp
			ldi		temp,0b00110001
			st		Y+,temp

			lds		r19,FLAG
			sbrc	r19,2
			rjmp	END_T1

			ldi		r16,250
			mov		r8,r16

			rjmp	END_T1

T1_LOW:
			ldi		temp,0b00001110
			st		Y+,temp
			ldi		temp,0b00111000
			st		Y+,temp

END_T1:

			in		temp,GIMSK
			sbr		temp,(1<<PCIE2)					; Разрешение прерываний кнопок калибровки
			out		GIMSK,temp

			lds		temp,FLAG
			cbr		temp,0x03			; Очищаем флаг
			sts		FLAG,temp
		
			ret


UR_INIT:
			ldi		r17,(MIN_UR)
			rcall	EEPROM_read
			sts		MIN_U,temp

			inc		r17
			rcall	EEPROM_read
			sts		(MIN_U+1),temp

			ldi		r17,(MAX_UR)
			rcall	EEPROM_read
			sts		MAX_U,temp

			inc		r17
			rcall	EEPROM_read
			sts		(MAX_U+1),temp
			
			lds		dd16uL,MAX_U
			lds		dd16uH,(MAX_U+1)

			lds		dv16uL,MIN_U
			lds		dv16uH,(MIN_U+1)

			sub		dd16uL,dv16uL
			sbc		dd16uH,dv16uH

			clr		dv16uH
			ldi		dv16uL,100

			rcall	div16u

			sts		RATIO,dres16uL

			mov		r17,drem16uL
			lsr		r17

			lds		r18,MAX_U
			lds		r19,(MAX_U+1)
			sub		r18,r17
			sbci	r19,0
			sts		MAX_U,r18
			sts		MAX_U+1,r19

			clr		r16
			lds		r18,MIN_U
			lds		r19,(MIN_U+1)
			add		r18,r17
			adc		r19,r16
			sts		MIN_U,r18
			sts		MIN_U+1,r19

			ret

Delay:							;процедура задержки

			subi	Razr0,1
			sbci	Razr1,0
			sbci	Razr2,0
			brcc	Delay
			ret

bin2bcd8:
			clr		temp		;clear result MSD
bBCD8_1:	subi	data,10		;input = input — 10
			brcs	bBCD8_2		;abort if carry set
			inc		temp		;inc MSD
			rjmp	bBCD8_1		;loop again
bBCD8_2:	subi	data,-10	;compensate extra subtraction
			ret


;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:		clr		drem16uL			;clear remainder Low byte
			sub		drem16uH,drem16uH	;clear remainder High byte and carry
			ldi		dcnt16u,17			;init loop counter
d16u_1:		rol		dd16uL				;shift left dividend
			rol		dd16uH
			dec		dcnt16u				;decrement counter
			brne	d16u_2				;if done
			ret							;    return
d16u_2:		rol		drem16uL			;shift dividend into remainder
			rol		drem16uH
			sub		drem16uL,dv16uL		;remainder = remainder - divisor
			sbc		drem16uH,dv16uH		;
			brcc	d16u_3				;if result negative
			add		drem16uL,dv16uL		;    restore remainder
			adc		drem16uH,dv16uH
			clc							;    clear carry to be shifted into result
			rjmp	d16u_1				;else
d16u_3:		sec							;    set carry to be shifted into result
			rjmp	d16u_1

			ret

;***************************************************************************
;*
;* "mpy8u" - 8x8 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two register variables mp8u and mc8u.
;* The result is placed in registers m8uH, m8uL
;*  
;* Number of words	:9 + return
;* Number of cycles	:58 + return
;* Low registers used	:None
;* High registers used  :4 (mp8u,mc8u/m8uL,m8uH,mcnt8u)	
;*
;* Note: Result Low byte and the multiplier share the same register.
;* This causes the multiplier to be overwritten by the result.
;*
;***************************************************************************

;***** Subroutine Register Variables

/*.def	mc8u	=r16		;multiplicand
.def	mp8u	=r17		;multiplier
.def	m8uL	=r17		;result Low byte
.def	m8uH	=r18		;result High byte
.def	mcnt8u	=r19		;loop counter

;***** Code


mpy8u:
			clr		m8uH		;clear result High byte
			ldi		mcnt8u,8	;init loop counter
			lsr		mp8u		;rotate multiplier
	
m8u_1:
			brcc	m8u_2		;carry set 
			add 	m8uH,mc8u	;   add multiplicand to result High byte
m8u_2:
			ror		m8uH		;rotate right result High byte
			ror		m8uL		;rotate right result L byte and multiplier
			dec		mcnt8u		;decrement loop counter
			brne	m8u_1		;if not done, loop more
			
			ret*/


EEWrite:
			sbic	EECR,EEPE			; Ждем готовности памяти к записи. Крутимся в цикле
			rjmp	EEWrite				; до тех пор пока не очистится флаг EEWE
			cli							; Затем запрещаем прерывания.

			ldi		r18, (0<<EEPM1)|(0<<EEPM0)
			out		EECR, r18

			out		EEAR,R17			; Загружаем адрес нужной ячейки
			out		EEDR,R16			; и сами данные, которые нам нужно загрузить
			sbi		EECR,EEMPE			; взводим предохранитель
			sbi		EECR,EEPE			; записываем байт
			sei							; разрешаем прерывания
			ret

EEPROM_read:
								; Wait for completion of previous write
			sbic	EECR,EEPE
			rjmp	EEPROM_read
								; Set up address (r18:r17) in address register
			out		EEAR,r17
								; Start eeprom read by writing EERE
			sbi		EECR,EERE
								; Read data from data register
			in		r16,EEDR
			ret

