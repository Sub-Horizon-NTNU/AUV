# Forslag til vision-pipeline for AUV og manipulator

Dato: 18. november 2026. Status: forslag, ikke ferdig implementasjon.

## 1. Formål og utgangspunkt

Vision skal finne relevante objekter og levere deres posisjon og eventuelt orientering relativt til kamerariggen. For manipulasjon må systemet også beskrive et gripepunkt og oppdatere forholdet mellom dette og griper under tilnærming.

Det lokale [kalibreringsnotatet](bakgrunn/KALIBRERING_START_HER.md) oppgir domehus som foreløpig hovedvalg, flat port som alternativ, Jetson Orin Nano som planlagt plattform og Light ESS / LightStereo-S som aktuelle dybdekandidater. Dette er arbeidsforutsetninger, ikke dokumenterte innkjøps- eller ytelsesbeslutninger.

**Anbefaling:** Behold stereoprinsippet, men få ett par og én gripeoppgave til å fungere før flere par integreres. Stereobasert lokalisering er brukt til AUV-manipulasjon i [MARIS-arbeidet](https://www.sciencedirect.com/science/article/pii/S0045790616302233). Vårt konkrete oppsett må likevel valideres separat.

## 2. Kalibrering før drift

Lås fokus, oppløsning og montering. Kalibrer intrinsics i luft, estimer faktisk husgeometri under vann med Calibmar og bestem kameraenes innbyrdes transformasjon. Bruk uavhengige kjente mål over arbeidsavstand og bildeflate til kontroll.

Dome og flat port må behandles forskjellig. En desentrert dome kan bryte pinhole-antakelsen. To kameraer bak én felles dome er ikke automatisk begge sentrert; geometrien må vurderes for hver optiske bane. [Dome-geometri](https://arxiv.org/abs/2108.06575), [Calibmar](https://arxiv.org/abs/2405.18018).

Light ESS og LightStereo forventer rektifiserte stereo-input. Det er derfor en **beslutningsport** om husmodellen kan gi en tilstrekkelig nøyaktig pinhole-tilnærming over vårt arbeidsområde. En vanlig 2D-remapping er ikke generelt en eksakt løsning for ikke-sentrale kameraer ved alle dybder. Dersom restfeilen er for stor: forbedre sentrering/montering eller bruk refraktiv korrespondansesøk og stråletriangulering, som krever mer utvikling.

Pinax er en mulig korreksjonsmetode for flat port innenfor metodens forutsetninger, ikke en dome-modell. Calibmar estimerer parametere; koblingen fra disse til kjørende dybdealgoritme er fortsatt en integrasjonsoppgave. [Pinax-artikkelen](https://www.dexrov.eu/wp-content/uploads/2018/10/Luczynski_OceanEngineering2017.pdf).

## 3. Pipeline under drift

1. **Opptak:** Felles eksponeringstrigger, tidsstempler og kontroll av manglende eller feilparede bilder. Foretrekk global shutter ved bevegelse.
2. **Geometri og preprocessing:** Last riktig kalibrerings-ID. Korriger og rektifiser med validert modell. Reskalering/beskjæring skal oppdatere intrinsics og eventuelle bildepunkter konsistent.
3. **To parallelle grener:** Beregn disparitet fra stereoparet, og finn objektmaske eller samsvarende nøkkelpunkter. Kjør begge i samme rektifiserte venstrebilde, eller transformer masken eksplisitt til dybdekartets koordinater.
4. **3D-estimering:** Forkast ugyldige dispariteter og usikre punkter. Bruk masken til å velge objektets punkter. Tilpass kjent geometri eller trianguler definerte gripepunkter.
5. **Sporing og kvalitetskontroll:** Oppdater objekt-ID, målestatus, målingens alder og usikkerhet. En predikert posisjon skal merkes som prediksjon, ikke som ny observasjon.
6. **Manipulatorgrensesnitt:** Transformer et gyldig gripepunkt til armens referanseramme og bruk gjentatte målinger til små korreksjoner. Ved tap av gyldig visuell måling må styringen håndtere dette eksplisitt.

Se [diagrammet](PIPELINE.md) for dataflyt. En objektdetektor gir ikke metrisk dybde alene. Midtpunktet i to deteksjonsbokser er heller ikke garantert samme fysiske punkt. Middelverdien av synlige 3D-punkter er ikke automatisk et fysisk objektsenter eller et gripepunkt.

For validert, sentral og rektifisert stereo med sammenfallende hovedpunkter er dybden `Z = fx * B / d`. Her må brennvidde og disparitet være i samme pikseloppløsning, og baseline i meter. Generelt brukes full rektifiseringsgeometri, inkludert eventuell forskjell i hovedpunkt, eksempelvis Q-matrisen. Refraktive kameraer krever riktig strålegeometri.

## 4. Objektmodell: velg enklest mulig

Kan objektene merkes, start med AprilTag/ArUco og et kjent gripepunkt relativt til merket. Har de enkel form eller tydelig farge, prøv klassisk bildebehandling. Bruk en liten segmenteringsmodell når variasjonen krever det. Merker og standardiserte håndtak har vært brukt i [undervannsmanipulasjon ved University of Michigan](https://gidobot.engin.umich.edu/research/automating-rov-manipulation).

Full 6D-pose trengs når griperens orientering er viktig. Symmetriske objekter kan ha tvetydig orientering; representer bare det som faktisk kan observeres. FoundationPose kan undersøkes senere for kjent CAD/referanser og brukbar dybde. Det er ikke nødvendig for alle gripeoppgaver. [FoundationPose](https://github.com/NVlabs/FoundationPose).

## 5. Relativ posisjon, SLAM og flere kameraer

Global AUV-pose er ikke nødvendig for en samtidig måling i kamerarammen. Armstyring trenger fortsatt forholdet mellom kamera, arm og griper. Armkinematikk og leddmålinger er normalt den enkleste veien; visjon kan korrigere restfeil.

SLAM kjøres som en separat, valgfri gren for kart og bevegelse. Hvis vi skal skille objektbevegelse fra kamerabevegelse eller akkumulere målinger over tid, trenger vi egobevegelse eller en tydelig relativ sporingsmodell. Ikke alle SLAM-systemer tar inn eksterne dybdekart; velg integrasjon etter deres faktiske inputkrav.

Flere faste stereopar kan transformeres til en felles riggramme med kjente extrinsics og tidskonsistente målinger. Manglende synsfeltoverlapp hindrer samtidig triangulering på tvers av par. Håndleddskamera legges først til dersom okklusjon eller nærpresisjon krever det.

## 6. Maskinvare og programvare

Orin Nano er en kandidat for en begrenset pipeline; vi har ikke målt at stereodybde, deteksjon og SLAM får plass samtidig. CPU/GPU deler minne, og modellens mellomresultater kommer i tillegg til vektene. Sammenlign end-to-end-latenstid, minne, strøm og temperatur på målplattformen før eventuell oppgradering til Orin NX.

Som illustrasjon gir fire 1080p-kameraer ved 30 FPS omtrent 746 MB/s rå RGB8 eller 249 MB/s Mono8, før protokolloverhead. Sjekk faktiske USB-kontrollere, CSI-laner, kameradrivere og carrier board. JPEG/H.264 kan redusere transport, men påvirker forsinkelse og bildekvalitet.

Jetson håndterer bildebehandling og modeller. En mikrokontroller kan håndtere trigger, watchdog og tidskritisk I/O. Sørg for varmeledning til trykkhuset og stabil strømforsyning. Optimaliser med TensorRT FP16 der kandidat og versjon støtter det; tren på separat GPU-maskin ved behov.

Isaac ROS, JetPack, CUDA, TensorRT, modellformat og eventuelle plugins må velges som en kompatibel, versjonslåst kombinasjon for Orin Nano. Ikke anta at nyeste pakke eller ferdig engine støtter vårt oppsett. [Isaac ROS stereopakke](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth).

## 7. Neste beslutning

Følg [testplanen](TESTPLAN.md): avklar arbeidsavstand og toleranse, valider optikken, sammenlign SGM/SGBM med Light ESS og LightStereo-S, og test én gripetilnærming med visuell feedback. Modellvalg og antall kameraer avgjøres av disse resultatene.
