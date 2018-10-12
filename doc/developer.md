# Developer Docs

This document describes information primarily for MuSHR developers. In MuSHR, we use "developer" to describe contributors to the main MuSHR repository (this repo), or to any of the main ROS packages referenced in our .rosinstall files, demos, or (in many cases) launch files. 

Use this document as a guide to understand code conventions, expectations about test automation, and templates for writing the `README.md` docs that must accompany any new code or launch file submission.

## Table of Contents
_table of contents goes here_

## Contributing to MuSHR

The MuSHR development process uses standard software engineering practices that ought to be familiar to most developers. In order to contribute, do the following:

* Fork the MuSHR repository.
* Read up on the [Gitflow Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow).
* Get acquainted with the MuSHR Stack:
	* Read the remainder of this writeup.
	* Read the `architecture.md` and `hardware.md` documentation files.
	* Complete some of the MuSHR tutorials.
* Get in touch with MuSHR developers about contributing!

## System Overview

The MuSHR system is distributed over a series of repositories, however the MuSHR main repository is the "central" repo wherein general systems assets, such as demos, launch files, assets, rosinstall files, documentation, and tutorials are located. The remaining ancillary repositories contain source code for the ros packages upon which MuSHR relies. These are referenced by the rosinstall files, and are also linked in `README.md` files in the `/launch` folder. 

This structure encourages users to use the main MuSHR repo a starting point for MuSHR-related projects, however some advanced users prefer the bare minimum robot code assets and these can accessed in the independent MuSHR rospackage repos. The following diagram ought clarify the repository structure:

![Repository Diagram](https://raw.githubusercontent.com/personalrobotics/mushr/development/doc/repo_overview.png)

## Hardware Management
_So you have a car. How do you maintain it? TODO: Patrick?._

## Style Guide

_TODO: Colin Summers_

## ROS Package Code Conventions

_Talks about how we organize ROS package repos. Expectations related to launch files, etc. that are not covered by the style guide._

## README.md Template

_TODO: Johan Michalove_

## Test Automation

_Coming soon to a repo near you!_

## Frequently Asked Questions

_So far, no questions have been asked frequently!_

