---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user:
command: /sp.constitution
labels: [constitution,spec-driven-development,rag-chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

```
Project: AI-Spec–Driven Technical Book with Embedded RAG Chatbot

Core Principles:
- Spec-driven development using Spec-Kit Plus
- Technical accuracy and no hallucinations
- Clear, developer-focused writing
- Fully reproducible build and deployment
- RAG responses strictly grounded in book content or user-selected text

Standards:

Book:
- Tools: Claude Code + Spec-Kit Plus
- Framework: Docusaurus
- Deployment: GitHub Pages
- Structured chapters, clear explanations, documented code

RAG Chatbot:
- Backend: FastAPI
- LLM: OpenAI Agents / ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- SQL DB: Neon Serverless Postgres
- Must cite source sections or return "not found"

Constraints:
- Free/open-tier infrastructure only
- Modular, documented, reproducible code
- No undocumented or proprietary APIs

Deliverables:
1. Docusaurus book source
2. Deployed GitHub Pages site
3. FastAPI RAG backend
4. DB schemas and vector indexing
5. Embedded chatbot UI

Success Criteria:
- Book builds and deploys successfully
- Chatbot answers accurately without hallucination
- User-selected text queries are correctly scoped
- Independent developer can reproduce the project
```

## Response snapshot

The project constitution has been successfully updated with the following content:

# AI-Spec–Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-Driven Development using Spec-Kit Plus
All development follows Spec-Kit Plus methodology; Specifications must be complete before implementation begins; All changes are guided by documented specs

### Technical Accuracy and No Hallucinations
All content must be factually accurate with no hallucinated information; RAG responses must cite source sections or return 'not found'; Strict grounding in book content or user-selected text required

### Clear, Developer-Focused Writing
Documentation must be clear and accessible to developers; Code examples must be well-documented and reproducible; All explanations must be structured and easy to follow

### Fully Reproducible Build and Deployment
All build processes must be reproducible by independent developers; Deployment procedures must be documented and tested; Infrastructure as code using free/open-tier services

### Modular, Documented, Reproducible Code
Code must be modular with clear interfaces; All components must be properly documented; Solutions must be reproducible without proprietary dependencies

### Free/Open-Tier Infrastructure Compliance
All infrastructure must use free/open-tier services only; No proprietary or paid APIs unless explicitly documented; Cost-effective solutions that remain within free tier limits

## Technology Stack and Standards

Book: Tools: Claude Code + Spec-Kit Plus; Framework: Docusaurus; Deployment: GitHub Pages; RAG Chatbot: Backend: FastAPI; LLM: OpenAI Agents / ChatKit SDKs; Vector DB: Qdrant Cloud (Free Tier); SQL DB: Neon Serverless Postgres

## Development Workflow and Quality Gates

Implementation of deliverables following spec-driven approach: Docusaurus book source, deployed GitHub Pages site, FastAPI RAG backend, DB schemas and vector indexing, embedded chatbot UI; All deliverables must meet success criteria including successful build/deployment, accurate responses without hallucination, and reproducibility

## Governance

All implementations must comply with core principles; PRs must verify adherence to no-hallucination requirement; Code reviews must check for reproducibility and use of free-tier infrastructure; All changes must be spec-compliant and properly documented

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16

## Outcome

- ✅ Impact: Project constitution updated with all specified principles and constraints
- 🧪 Tests: N/A (Documentation update)
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: Next steps include creating feature specifications based on the established constitution
- 🧠 Reflection: Constitution successfully captures all core principles and constraints from the project specification

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating feature specifications based on the constitution