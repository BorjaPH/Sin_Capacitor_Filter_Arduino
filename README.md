# Sin_Capacitor_Filter_Arduino

Using a capacitor filter in combination with a PWM signal generated through pin OC2A, it generates a sinusoidal signal with a period Ts:

                                                            Vout≈V⋅sen(2π/Ts*t)

Being "V" the maximum voltage (in milivolts). In this program "t" should be less or equal to 1 sec.

Apart from this, it has the command "period [miliseconds]" to change the period of "Ts".

The hardware montage is the same as described in the schematic below:

![Esquemático_2](https://user-images.githubusercontent.com/68208538/135026542-fd15a03a-b985-4058-9605-36c1a4df14c2.PNG)
