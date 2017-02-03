# 220C Detektor for varmeelement

Jeg ønsker å bruke røykskapet også for tørking av kjøtt. Hvis det skal skje om vinteren ønsker jeg å unngå at det fryser. Derfor har jeg installert et varmeelement på 100W, som styres av en termostat. For å finne ut hvor mye strøm som går med til denne oppvarmingen ønsker jeg å detektere når varmeelementet er på og når det er av, og så beregne strømforbruket ut fra dette.

Jeg må derfor koble 220V-signalet til varmeelementet inn på arduinoen. Dette gjør jeg via en [HLK-PM03](http://www.ebay.com/itm/272252759542?_trksid=p2057872.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT) 220V AC til 5V DC omformer. Jeg skal vurdere å også bruke en optocoupler for å være sikker på at forstyrrelser på 220V-nettet ikke ødelegger arduinoen.


Min optocoupler er en EL817 ([datasheet](http://www.everlight.com/file/ProductFile/EL817.pdf)). Spenning inn skal være 1.2V, og den trekker 60 mA. Motstanden mellom anode og 5V skal da være:

R = U / I = (5V-1.2V) / 0.06A = 63 ohm.

Hvis jeg derimot bruker en 3.3V kilde skal motstanden være:

R = U / I = (3.3V-1.2V) / 0.06A = 35 ohm.
