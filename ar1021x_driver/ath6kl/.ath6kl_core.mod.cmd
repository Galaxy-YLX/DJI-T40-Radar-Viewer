savedcmd_ath6kl_core.mod := printf '%s\n'   debug.o hif.o htc_mbox.o htc_pipe.o bmi.o cfg80211.o init.o main.o txrx.o wmi.o core.o recovery.o | awk '!x[$$0]++ { print("./"$$0) }' > ath6kl_core.mod
