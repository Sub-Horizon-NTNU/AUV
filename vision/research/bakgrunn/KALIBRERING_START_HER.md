# Undervannskalibrering med Calibmar

## Foreløpig hovedvalg: dome, med flat port som alternativ

Prosjektet vurderer foreløpig domehus som hovedvalg for kameraet (presisert av bruker 18. november 2026). Valget er ikke endelig: hus med flat port kan være mer egnet for enkelte kameraer og plasseringer. Velg optisk modell per kamera og port, ikke én felles modell for hele roboten. Calibmars domekalibrering er hovedsporet for dome; Calibmar støtter også flat port. Pinax kan vurderes for flate porter innenfor metodens forutsetninger og skal ikke brukes direkte som korreksjonsmodell for domen.

Hentet 18. november 2026 fra forfatternes offisielle repositorium:
https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar

Commit: 5c5a8200b42b22bac4e1ff62d066c1373a5ce57b
Artikkel: A Calibration Tool for Refractive Underwater Vision, arXiv:2405.18018v2.

## Innhold

- `calibmar/src`: kildekode for verktøyet.
- `calibmar/lib`: medfølgende bibliotekkode, inkludert COLMAP.
- `calibmar/data/validation/Rendered`: simulerte bilder for kamera, dome, flat port og stereo.
- `calibmar/data/validation/Tank`: virkelige bilder fra vanntank, inkludert opptak i luft, dome og flat port.
- `calibmar/data/validation/README.md`: forklaring av datasettet. Undermappene inneholder også kalibreringsrapporter og loggfiler med fasit der den er kjent.
- `calibmar/doc/source`: brukerveiledninger og rapportformat.
- `calibmar/LICENSE.txt`: lisens; se også lisenser for medfølgende biblioteker.

Repositoriet er klonet med Git og Git LFS slik at bildene lastes ned som faktiske filer. Programmet er ikke bygget eller installert, og er ikke integrert i vision-pipelinen.

Dokumentasjon: https://mdskiel.github.io/calibmar_documentation/
Ferdige Windows/Linux-programmer: https://cau-git.rz.uni-kiel.de/inf-ag-koeser/calibmar/-/releases

## Anbefalt første kalibrering av vårt utstyr

1. Velg og lås fokus, zoom, oppløsning og bildebehandling som påvirker geometrien. Unngå autofokus og elektronisk stabilisering under målinger.
2. Kalibrer kameraets indre parametere i luft med et stivt, flatt sjakkbrett med nøyaktig kjent rutestørrelse.
3. Monter kameraet fast i undervannshuset. Samle minst omtrent 20 undervannsbilder med variasjon i brettets vinkel, avstand og plassering over bildeflaten.
4. Importer luftkalibreringen i Calibmar og velg modell for den faktiske porten. For dome: oppgi materialegenskaper, radius og glasstykkelse, og beregn kameraets forskyvning fra domens sentrum. For flat port: oppgi materialegenskaper og tykkelse, og estimer vinduets normal og avstand til kameraet.
5. Ved stereo: kalibrer også forholdet mellom kameraene. Verifiser at den videre stereoalgoritmen faktisk støtter valgt refraktiv modell.
6. Lagre YAML og kontroller målte avstander mot uavhengige, kjente lengder ved flere avstander og ute ved bildekantene.
7. Skal kameraet brukes sammen med IMU eller robotkoordinater, må transformasjonene mellom disse også bestemmes. Dette følger ikke automatisk av huskalibrering.

Referansebrettet brukes under kalibrering, og trenger ikke være synlig under oppdrag. Kalibrering gir ikke i seg selv robotens globale posisjon.

## Når må det gjentas?

Bevegelse av hele den stive roboten endrer normalt ikke kalibreringen. Kontroller eller kalibrer på nytt etter endring av fokus/zoom, kamera- eller portmontering, støt, eller annen endring av optisk geometri. Ny oppløsning/beskjæring krever konsistent justering av parametere eller ny kalibrering. Trykk kan deformere utstyr; valider ved relevante driftsforhold. Endrede vannforhold kan kreve oppdatert brytningsindeks i modellen.

## Bruk sammen med deteksjon og sporing

En aktuell kjede er: tidsstemplet bilde -> objektdeteksjon/segmentering -> velg relevante bildepunkter -> refraktiv kameramodell gir stråler i vann -> dybde fra stereo, flere syn eller kjent geometri -> kombiner med robotpose og spor objektet i 3D.

Kalibrering forbedrer grunnlaget for retning, triangulering og geometrisk konsistens. Den forbedrer ikke automatisk et nevralt nettverks gjenkjenning, og gir ikke alene avstand fra ett bilde av et objekt med ukjent størrelse.

En flat port er generelt ikke en ideell pinhole-modell. Vanlig linse-undistortion med OpenCV er derfor ikke nødvendigvis nok. Den videre geometrikoden må bruke husparametrene og en refraktiv modell, eller en dokumentert og validert tilnærming for aktuelt avstandsområde.

Ta hensyn til eksponeringstid, bevegelsesuskarphet, rolling shutter, tidssynkronisering og endringer i sikt. Kamerabevegelse må skilles fra objektbevegelse, for eksempel med visuell odometri og IMU, eventuelt andre navigasjonssensorer.

Dette er en anbefalt anvendelse for prosjektet, ikke en dokumentert nøyaktighetsgaranti for vårt utstyr. Testbilder fra artikkelen erstatter ikke kalibrering av våre egne kameraer.

## Pinax og arbeidsdeling mellom kalibrering og drift

Referanse: Luczynski, Pfingsthorn og Birk, «The Pinax-Model for Accurate and Efficient Refraction Correction of Underwater Cameras in Flat-Pane Housings» (Ocean Engineering, 2017). [Bevart kopi av artikkelen](../references/Luczynski_OceanEngineering2017.pdf).

Pinax og Calibmar behandler samme grunnproblem, men har forskjellig rolle og virkeområde. Calibmar estimerer kamera- og husparametere og støtter både dome og flat port. Pinax gir effektiv bildekorreksjon for flat port under bestemte geometriske forutsetninger, blant annet at kameraet er nær vinduet. Pinax bruker et forhåndsberegnet oppslagskart og vannets brytningsindeks. Artikkelen sammenligner også feil i triangulerte 3D-punkter i stereoforsøk.

Ideen om Calibmar ved kalibrering og Pinax under drift gjelder ikke direkte for dome. Vi kan bruke prinsippet om forhåndsberegning, men da må modellen og valideringen være for dome. Pinax-artikkelen beholdes som faglig referanse og mulig metode for fremtidige kameraer med flat port, ikke som valgt dome-implementasjon. Bruk av Calibmar-resultater i Pinax er ikke en ferdig integrasjon; parameterbehov og forutsetninger må kontrolleres.

Ved valg av port per kamera bør vi veie synsfelt og arbeidsavstand mot plass, montering, trykkbelastning, produksjon og krav til målenøyaktighet. Dome er avhengig av god sentrering for å utnytte de optiske fordelene; flat port kan være mekanisk enklere, men gir lysbrytning og redusert synsfelt under vann. Registrer porttype og egen kalibrering for hvert kamera, og kalibrer forholdet mellom kameraene dersom de skal brukes sammen.

### Før oppdrag og ved endringer

- Plasser kameraets inngangspupill så nær domens krumningssenter som praktisk mulig. Ideell sentrering reduserer retningsendring fra lysbrytning, men faktisk oppsett må måles og testes.
- Bruk Calibmar til luftkalibrering og undervannsestimering av dome-forskyvningen. Lagre resultater per fast kamera-, linse- og husoppsett; full kalibrering trengs normalt ikke ved hver oppstart.
- Verifiser på uavhengige kjente mål over relevant avstandsområde og bildeflate. Gjør en enkel kontroll før oppdrag og ny kalibrering ved endret optisk geometri.

### Under oppdrag

- Last lagrede parametere og bruk dem i en dome-kompatibel geometrimodell.
- For objektdeteksjon kan originalbildet brukes dersom modellen er trent for dette. Beregn deretter refraktive synsstråler for bare de punktene som trengs til 3D-sporing.
- Ved behov kan piksel-til-stråle-data forhåndsberegnes for fast optikk og brytningsindeks. For en desentrert dome må både strålens startpunkt og retning i vann bevares; en enkel 2D-bildekorreksjon til et ideelt pinhole-kamera er ikke generelt eksakt for alle dybder.
- Bruk vanlig pinhole-/stereogeometri bare dersom den er validert som tilstrekkelig nøyaktig for riggen og arbeidsavstandene. Ved stereo må begge kameraenes optiske baner kontrolleres; sentrering av ett kamera betyr ikke at begge er sentrert.
- Beregn robotens bevegelse fortløpende separat fra den faste optiske kalibreringen.

Dette er en arkitekturanbefaling. Dome-korreksjon, oppslagskart og kobling til vision-modellen er ikke implementert eller ytelsesmålt ennå.

## Kandidater til stereodybde i vision-pipelinen

Prosjektet planlegger stereodybde og visuell SLAM på Jetson Orin Nano. Light ESS eller LightStereo-S kan brukes som dybdekomponent, forutsatt at tester på vår rigg viser tilstrekkelig nøyaktighet, hastighet og minnekapasitet. Modell er ikke endelig valgt eller implementert.

| Kandidat | Aktuell bruk |
| --- | --- |
| NVIDIA Light ESS | Første kandidat for ferdig Jetson-/TensorRT- og ROS 2-integrasjon via Isaac ROS. Tar inn rektifiserte RGB-bildepar på 480 x 288 og gir disparitet samt et sikkerhetskart som kan brukes til å forkaste usikre punkter. |
| LightStereo-S | Alternativ med liten modell og tilgjengelig forskningskode for egen trening og tilpasning til undervannsdata. Gir disparitet. |

Forslag til dataflyt: synkroniserte stereobilder -> optisk korreksjon og rektifisering -> Light ESS eller LightStereo-S -> disparitet omregnes til metrisk dybde med kalibreringen -> kartlegging og objektposisjonering. Visuell/stereo-inertiell SLAM kan kjøre parallelt. Integrasjonen avhenger av om valgt SLAM-system bruker bildene direkte eller støtter eksternt dybdekart; tett dybde er ikke et krav for all SLAM.

For dome og eventuell flat port må vi verifisere at rektifiseringen er god nok for modellens søk langs bilderader over relevante arbeidsavstander. Modellene erstatter ikke kamera- og huskalibrering.

Ytelsestall fra RTX 3090 eller Jetson AGX Orin er ikke garantier for Orin Nano. Sammenlign kandidatene med en klassisk stereometode på egne tankopptak. Mål avstandsfeil, gyldig dybdedekning, tidsstempelkonsistent forsinkelse og minnebruk mens SLAM og objektdeteksjon kjører samtidig. Start med moderat oppløsning og vurder TensorRT FP16 der det støttes. Undervannsytelse er foreløpig ikke validert.

Referanser:

- Light ESS modellkort: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/models/dnn_stereo_disparity
- Isaac ROS-integrasjon: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth
- ESS forskningsbakgrunn: «On the Importance of Stereo for Accurate Depth Estimation» (2018), https://arxiv.org/abs/1803.09719. Dette er bakgrunn, ikke en full beskrivelse av dagens Light ESS.
- LightStereo-artikkel: «LightStereo: Channel Boost Is All You Need for Efficient 2D Cost Aggregation», https://arxiv.org/abs/2406.19833
- LightStereo-kode: https://github.com/XiandaGuo/OpenStereo
