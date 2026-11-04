---
name: crushopenitems
description: Work through every open item in the ac3 RC foiling sailboat repo that Claude can close without Lukas, then report what is left for him. Finds stale or mismatched numbers by re-running the script chain against specs/, resolves open doc/analysis items, logs each change in docs/build-log.md, and commits on a branch. Use whenever the user says "crush open items", "clear the open items", "what's still open and fix what you can", "close out the open confirmations", "check the specs against the scripts and fix drift", or asks for a cleanup/sweep of ac3 specs, even if they don't name the skill.
---

# crushopenitems

Goal: leave ac3 with as few open items as honestly possible, and a short, accurate list of what only Lukas (or a bench/on-water test) can close. Honest matters more than empty: an item closed on an assumption is worse than an item left open and labelled.

Read `CLAUDE.md` first if it is not already in context. Its working rules (numbers must be real, each fact in one place, build-log entry when a documented number changes, never promote PROPOSED to DECIDED) bind everything below.

## 1. Set up

- Work from `/home/user/ac3`. If `.venv` is missing, run `./setup.sh`. Scripts must be run from `scripts/` with `../.venv/bin/python` (they load `arm.py` and `arm_pts.csv` from the working directory).
- `git fetch origin main`, then create a branch off `origin/main` (for example `crush-open-items-<date>`). Never commit to `main`.
- Send script output to the scratchpad, not the repo.

## 2. Inventory

Build one list of candidate items from all of these:

- **Tagged items**: grep `specs/` for `OPEN` and `PROPOSED`; read `08-open-confirmations.md`, the risks in `09-10-weight-and-risks.md` and the decision-log rows in `00-concept-and-decision-log.md`.
- **Notes left by earlier passes**: "not fixed", "noticed", "not checked" lines in `docs/build-log.md`.
- **Drift between scripts and specs**: re-run the chain and compare every documented figure. Run `opt.py` (library, prints nothing) through `r250c.py`, `top_reach.py`, `cg_layout.py`, then `sail_rescan.py`, `docnums.py`, `joint_straight.py`, `breakeven.py`, and `strength_v4.py` / `topreach_checks.py` where specs cite them. The root `README.md` says which scripts reproduce which section. Compare with the matching spec table or sentence, not from memory.
- **Unsourced figures**: numbers a script prints that no file mentions. Search `specs/`, `docs/mechanisms/`, `docs/tests/` and the README before calling one unverified.
- **Git state**: unmerged branches and open PRs (use `gh api repos/LukasWolfgangg/Ac3/...`; GraphQL is blocked in these sessions, so `gh pr create` fails and the REST API is needed).

## 3. Sort every item into one of three bins

- **Claude can close**: stale or mismatched numbers, a figure that can be recomputed or sourced, a doc sentence that restates a fact owned by another file, a check a script can run, a missing cross-reference. Do these.
- **Needs Lukas's decision**: anything that would turn PROPOSED into DECIDED, choices between options (roll-path fix, pin and wire materials, ply orientation), trade-offs, spending foil.tools credits (`analyze_wing`, `analyze_trim` and `optimize_airfoil` are credit-metered; ask first).
- **Needs a test or outside input**: coupon test, twist test of the first arm, on-water drag runs, supplier confirmations (for example HPC wheel face width), anything living only in Fusion cloud or the Drive docs. Do not substitute an assumption.

Do not touch bins two and three except to make the item's description precise (what exactly is needed, and what it blocks).

## 4. Close the items in bin one

One item at a time, so each change is traceable:

1. Find the source of truth (usually a `specs/` file) and change it there. Other files point to it; do not restate numbers (rule 4). Remove outdated text rather than keeping it beside the new text.
2. If a script output changes a documented number, recompute all downstream numbers before moving on (rule 3), update the doc in the same change and append a `docs/build-log.md` entry (rule 10). Entries state what was wrong, the old and new values, and what was not changed. Date them with today's date.
3. Keep status tags as they are. A number you recomputed is still PROPOSED.
4. Re-run the script(s) the change touches and confirm they now agree with the doc.
5. Leave frozen records alone (Foilarm_v5 and Gimbal D notes in `cad/`) and do not invent figures; if nothing in the repo supports a number, report it unverified.

## 5. Finish

- Review `git diff`; commit with a message that says what drifted and why. Push with `git push -u origin <branch>`. The proxy can refuse branch deletion, so do not try to clean up remote branches. Open a PR only if the user asks (REST API, as above).
- Final report, direct and short, in three lists:
  - **Closed**: item, old value to new value, file.
  - **Needs your decision**: the item, the options, and your recommendation.
  - **Needs a test or outside input**: what, why it matters, and which margins hang on it.
- Say what you did not check. If a pass found nothing to fix, say that plainly instead of padding.
