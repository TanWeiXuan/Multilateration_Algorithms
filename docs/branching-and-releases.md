# Branching and Releases

## Branch Contract

- `main` is the only development and review branch. It contains the native CLI, web app, workflows, and documentation.
- `pages` is a deployment pointer. It must contain no independent commits and must only move by fast-forwarding to a tested commit already on `main`.
- Feature, fix, and documentation branches start from `main` and open pull requests back to `main`.

Protect both branches where repository settings permit it. Require native and web validation on `main`, disallow direct pushes to `pages` except for maintainers performing a deployment, and disallow force-pushes.

## Deploying

After the selected `main` commit passes CI and review:

```bash
git fetch --prune origin
git switch main
git pull --ff-only origin main
git switch pages
git merge --ff-only main
git push origin pages
```

The push to `pages` starts `.github/workflows/pages-webapp.yml`. Record the deployed commit SHA in the release or deployment notes.

Verify ancestry before and after promotion:

```bash
git merge-base --is-ancestor pages main
git rev-parse main pages
```

Immediately after a promotion, both hashes should match. At other times, `pages` may trail `main`, but it must remain an ancestor of `main`.

## Rollback

Do not rewrite `pages` or create a one-off rollback commit there. Revert the faulty change on `main`, validate the revert, then fast-forward `pages` to the new revert commit using the normal deployment procedure. This preserves an auditable history.

## Recovering from Divergence

If `pages` contains a commit that is not on `main`, stop deployments. Identify the unique commits with:

```bash
git log --oneline --left-right main...pages
```

Integrate any valid work into `main` through a reviewed merge or cherry-pick. Only after `pages` is again an ancestor of `main` should it be advanced. Avoid force-pushing unless repository history is provably unrecoverable and all maintainers agree.
