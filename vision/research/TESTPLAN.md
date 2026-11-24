# Første testplan

Dette er foreslåtte forsøk og akseptansekriterier som må tallfestes. Tabellen inneholder ingen målte resultater.

## 1. Avklar krav før modellvalg

| Krav | Må bestemmes av teamet |
| --- | --- |
| Oppgave | Objekttyper, om de kan merkes, ønsket gripepunkt |
| Arbeidsområde | Minste/største avstand, synsfelt og okklusjon |
| Toleranse | Tillatt posisjons- og orienteringsfeil ved grep |
| Bevegelse | Relativ hastighet og tillatt alder på målingen |
| Kapasitet | Antall samtidige par, ønsket oppdateringsfrekvens |
| Drift | Sikt, lys, driftsdybde, strøm- og varmebudsjett |

En første latenstidsgrense kan utledes fra tillatt bevegelsesfeil delt på relativ hastighet. Dette er bare én del av feilbudsjettet; kalibrering, dybdestøy og armfeil må få egne marginer.

## 2. Test geometri før nettverk

- Kalibrer egne kameraer og faktisk dome/flat port. Lagre råopptak, parametere, enheter, transformasjonsretning og kalibrerings-ID.
- Bruk uavhengige kontrollmål ved flere avstander, i sentrum og ved bildekantene.
- Kontroller rektifiseringsrestfeil og metrisk feil. Lav reprojeksjonsfeil alene er ikke tilstrekkelig.
- For dome: vurder begge optiske baner, særlig ved felles dome.
- Hvis vanlig rektifisert stereo ikke holder toleransen: rett optikken eller bruk refraktiv geometri før videre modellvalg.

## 3. Sammenlign dybdekandidater

Kjør SGBM/SGM, Light ESS og LightStereo-S på samme opptak. Registrer faktisk modellinput, oppdatert kalibrering og presisjon. Test klart vann, svakere lys, partikler, blanke/tekstursvake objekter og delvis tildekking av griper. Dokumenter også feiltilfeller og forkastede punkter.

Mål median og 95-persentil av absolutt 3D-feil mot uavhengig referanse, gyldig punktandel i objektmasken, målestabilitet og falskt gyldige målinger. En modell som bare beholder få enkle punkter må ikke vurderes utelukkende på feilen til disse.

## 4. Mål hele kjeden på Jetson

Mål fra eksponering til tilgjengelig objektmåling, med konsistente klokker. Logg latenstidens median/95-persentil, levert målefrekvens, droppede bilder, topp minnebruk, temperatur og effektmodus. Kjør lenge nok til å oppdage termisk nedklokking; begrunn varigheten med stabilisert temperatur.

Test først stereo alene, deretter med objektmodell, og til slutt med eventuell SLAM. Referansemålinger på desktop-GPU eller AGX Orin kan ikke erstatte denne testen.

## 5. Test griping med visuell feedback

Start med kjent objekt, fast definert gripepunkt og én arm i rolig tank. Mål gjentatte forsøk, treffandel og feil ved tilnærming. Test deretter AUV-bevegelse og planlagt arbeidsmiljø. Tap av objekt, gamle målinger og lav kvalitet skal gi eksplisitt status til styringen, ikke videre bruk av gammel posisjon som fersk.

## Beslutningsporter

1. **Optikk godkjent:** Uavhengige målinger holder det avklarte feilbudsjettet.
2. **Pipeline godkjent:** Nøyaktighet, gyldig dekning og latenstid holder samtidig på målplattformen.
3. **Gripeforsøk godkjent:** Gjentatte forsøk fungerer også med relevante forstyrrelser.
4. **Utvidelse begrunnet:** Flere kameraer, større Jetson eller tyngre modell innføres for en dokumentert begrensning.

## Resultatlogg

Bruk [CSV-malen](benchmarks/resultater.csv). Ett sett resultater må vise til datasett/økt, referansemålemetode og kode-/modell-/kalibreringsversjon. Ingen opptak eller faktiske kalibreringsfiler følger med denne rapporten.
