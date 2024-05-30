# meteomodem_m20_custom_firmware

Alternative firmware for new models of radiosondes : Meteomodem M20 (including STM32L051R6T6 ADF7012 XM1110)

Made with STM32CubeIDE

Currently working on, I have been able to send a valid POCSAG frame for the moment.

More updates coming later on this Youtube playlist :

https://youtube.com/playlist?list=PLKM5FNxF7v42fTIaH2O8OKJLjFLZwaq5I

You can join us on Discord if you wanna talk about this project : https://discord.gg/QCYmcGjAeA


<br><br>
Thanks to:<br>
- Salim CHIAD F4IFB ([@schiad](https://github.com/schiad)) who encouraged me a lot to start this project, he made some very useful functions like sending data to the transmitter registers or a function that translate frequency into the coorresponding register instruction.<br>
- Jarek Surmacz ([@infrat](https://github.com/infrat)) who I think made [the first opensource project on the M20](https://github.com/infrat/m20-playground), a simple thing that collect GPS data and print it to serial TX, it was inspiring for our first sandboxes.<br>
- Tobias and Matthias who did an incredible M20 Reverse Enginering work on [their personnal blog](https://www.egimoto.com).<br>
The magical PDF: [PDF_M20_REVERSE](https://www.egimoto.com/dwld/17528ed1858138.pdf)
