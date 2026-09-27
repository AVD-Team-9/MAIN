# Team HW3 Overleaf Project

This project compiles one team PDF containing team-level Sections 1–6 followed by a separate 1–2 page contribution for each Team 9 member.

## Project Folders

The project id divided into folders, the main tex and other supporting files. See the image below:

<img width="256" height="248" alt="image" src="https://github.com/user-attachments/assets/f9e715a9-106d-4caf-b43b-1c6b2789b45b" />


## Upload to Overleaf

1. In Overleaf, choose **New Project → Upload Project**.
2. Upload the ZIP file.
3. Confirm `main.tex` is selected as the main document.
4. Use pdfLaTeX. Overleaf should automatically run BibTeX when citations or the bibliography change; if references lag, choose **Recompile from scratch**.

## Upload MATLAB figures as PNG

All image files belong in the project-level `figures/` folder.

1. In MATLAB, create and format the final plot with readable labels, units, legend, and annotations.
2. Export a 300 dpi PNG. Example:

```matlab
set(gcf,'Color','w');
exportgraphics(gcf,'henry-swafford-trade.png','Resolution',300);
```

3. In Overleaf, open the `figures` folder and click **Upload**. Select the exported PNG.
4. Make the uploaded filename exactly match the command in the member file. Example:

```latex
\reportfigure{henry-swafford-trade.png}
{Effect of design range on takeoff gross weight.}
{fig:henry-swafford-trade}
```

The custom `\reportfigure` command automatically searches `figures/` when the team report is compiled and `../figures/` when a member file is compiled independently. Until the named PNG is uploaded, the PDF displays a placeholder instead of failing.

Recommended MATLAB export size for a single-column report figure:

```matlab
set(gcf,'Units','inches','Position',[1 1 7.0 4.2]);
set(gca,'FontSize',11);
exportgraphics(gcf,'student-name-trade.png','Resolution',300);
```

Do not paste screenshots of MATLAB figures. Export the plot directly so text and lines remain clear.

## Individual member files

Each person has a separate file in `members/`:

- `henry-swafford.tex` — Team Lead
- `zilmond-strader.tex` — Chief Engineer
- `nathanael-virgil-fenelus.tex` — Aerodynamics
- `marcos-santana.tex` — Propulsion
- `adetomiwa-debo-lawal.tex` — Propulsion / Stability and Controls
- `aidan-linebarger.tex` — Avionics / Subsystems
- `zian-niu.tex` — Avionics / Configurations
- `logan-bobrow.tex` — Structures
- `kennedy-oti.tex` — Structures / Mass Properties
- `jonah-chase.tex` — Mission / Cost Analysis

Members should only edit their own file and upload their uniquely named PNG to `figures/`. Each file can compile independently through Overleaf's **Compile this file** option and is already included in `main.tex`.

## References and BibTeX

The bibliography file is `references.bib` in the same folder as `main.tex`. The team RFP and GitHub repository are cited in Sections 1 and 4, so the References section displays immediately.

Add a source to `references.bib`, then cite it in text:

```latex
The selected method is appropriate for preliminary sizing \cite{sourceKey}.
```

The end of `main.tex` contains:

```latex
\bibliographystyle{ieeetr}
\bibliography{references}
```

If a newly added source does not appear, confirm its citation key is used, check the `.bib` syntax, and select **Recompile from scratch**. BibTeX normally omits entries that are never cited.

## GitHub repository

Browser URL: <https://github.com/SantanaMarcoss/Team9>

Clone command:

```bash
git clone https://github.com/SantanaMarcoss/Team9.git
```

Before submission, replace every `COMMIT-HASH` and code path with a permanent GitHub link. Add `Krishbhatt01` and `qmciver` as repository collaborators, and make the repository README map students to code, figures, and commit permalinks.

## Final checks

Search the project for `TBD`, `COMMIT-HASH`, and `path/to/file.m`. Confirm each trade plot contains labeled axes and units, threshold/objective markers where applicable, frozen baseline, recommendation, and a readable caption. Keep each individual contribution between one and two pages.

## AI and course policy

This formatting template was AI-assisted. Follow the course policy for this homework. If content is later reused in the AIAA competition report, review the applicable AI-disclosure rules and include any required disclosure. Team members remain responsible for the technical work, calculations, citations, and submitted writing.
