#!/bin/zsh

if ! which conda >/dev/null; then
	echo "Error: conda is not installed." >&2
	exit 1
else
	conda create -y --name slpy python=3.11.2
  # conda activate cannot be used in scripts; find the executable and source it
  if ! source $HOME/anaconda3/bin/activate slpy; then
    if ! source $HOME/miniconda3/bin/activate slpy; then
      echo "Error: Failed to activate the conda environment. Please activate it manually (e.g., run conda activate slpy) and run poetry install." >&2
      exit 1
    fi
  fi
fi
# Install the dependencies with Poetry
if ! which poetry >/dev/null; then
  # Prompt the user to install Poetry if not installed
	echo "Error: Poetry is not installed. Do you want to install it? [Y/N]: "
  read install_poetry
    if [[ "$install_poetry" = "N" ]] || [[ "$install_poetry" = "n" ]]; then
      echo "Error: Failed to install the environment with Poetry." >&2
      exit 1
    else
      curl -sSL https://install.python-poetry.org | python3 -
    fi
else
	echo "Building the environment with Poetry..."
  # poetry insall can fail for older versions; prompt the user to update Poetry
  if ! poetry install; then
    echo "Error: Failed to install the environment with Poetry. Do you want to update Poetry and try again? [Y/N]: "
    read update_poetry
    if [[ "$update_poetry" = "N" ]] || [[ "$update_poetry" = "n" ]]; then
      echo "Error: Failed to install the environment with Poetry." >&2
      exit 1
    else
      poetry self update
      poetry install
    fi
  fi
fi
# Test the installation
echo "Do you want to proceed with an installation testing? [Y/N]: "
read -t 5 answer
if [[ "$answer" = "N" ]] || [[ "$answer" = "n" ]]; then
	echo "In case you want to run the test later, type pytest"
	exit 0
else
	pytest
fi
