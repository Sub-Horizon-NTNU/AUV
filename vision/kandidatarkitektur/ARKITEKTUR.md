# Kandidatarkitektur for vision

**Dato:** 24. november 2026.

**Status:** Arbeidsplan for første oppsett. Ikke implementert eller ytelsesvalidert.

## Mål

Estimere posisjon og relevant orientering for et oppgaveobjekt, og levere et definert gripepunkt til manipulatorstyringen. Første oppsett omfatter ett stereopar og ett kjent objekt.

## Planlagt oppsett

| Del | Arbeidsplan | Gjenstår å avklare |
| --- | --- | --- |
| Kamera | Ett fast, synkronisert stereopar; global shutter er ønsket | Kameramodell, baseline, objektiv, fokus og tilkobling |
| Kamerahus | Dome som foreløpig hovedspor; flat port er et alternativ | Porttype og mekanisk oppsett per kamera |
| Datamaskin | NVIDIA Jetson Orin Nano Super 8 GB | Strømforsyning, varmeledning og kompatible programvareversjoner |
| Kalibrering | Calibmar for kamera, hus og stereo | Kalibreringsopptak, geometri og kontrollmålinger |
| Stereodybde | Ferdigtrent NVIDIA Light ESS via Isaac ROS/TensorRT | Kvalitet i vann, rektifisering og samlet ressursbruk |
| Objektlokalisering | Enkel geometri eller en liten segmenteringsmodell | Objekter, metode og eventuell treningsdata |
| Griping | Gripepunkt i armens koordinatsystem, oppdatert under tilnærming | Kamera-til-arm-transformasjon og krav til presisjon |

Undervannsfinjustering av Light ESS inngår som en uavklart mulighet, ikke som en forutsetning for første oppsett. Flere stereopar, SLAM og tyngre posemodeller er mulige senere utvidelser.

## Kalibrering og oppstart

Grunnkalibreringen gjennomføres ved etablering av et fast kamera-, linse- og husoppsett: kameraegenskaper i luft, husgeometri under vann og kameraenes innbyrdes transformasjon. Resultatene lagres med en kalibrerings-ID.

Ved vanlig oppstart lastes lagret kalibrering. En enkel kontroll mot kjente mål brukes til å oppdage avvik. Full kalibrering gjentas ved relevante endringer i optikk eller montering. Kalibreringen må også kobles til korrekt rektifisering under drift; en eksportert Calibmar-fil alene fullfører ikke dette.

## Første gjennomføring

1. Fastsett nødvendig arbeidsavstand og tillatt posisjons- og orienteringsfeil.
2. Monter og kalibrer ett stereopar; kontroller mot uavhengige mål i vann.
3. Kjør Light ESS på opptak og mål feil og gyldig dybdedekning.
4. Koble objektlokalisering til dybden, og mål hele kjedens latenstid på Jetson.
5. Test én gripetilnærming med gjentatte visuelle målinger.

Beståttkriterier tallfestes før testing. Ingen bestemt nøyaktighet, oppdateringsfrekvens eller gripekapasitet er dokumentert for riggen ennå.

## Grunnlag

- [Pipeline-rapport](PIPELINE.md)
- [Research, alternativer og kilder](../research/README.md)
- [Testgrunnlag](../research/TESTPLAN.md)
- [NVIDIA ESS](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/dnn_stereo_disparity), [Isaac ROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)
- [Calibmar: huskalibrering](https://mdskiel.github.io/calibmar_documentation/housing_calibration.html), [stereokalibrering](https://mdskiel.github.io/calibmar_documentation/stereo_calibration.html)
