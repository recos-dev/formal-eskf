# Specification Index

Read the relevant requirement and its declared dependencies. Directory placement
identifies ownership, not requirement derivation. Requirement definitions, design
explanations, deployment profiles and verification policy have separate owners.

Use [TEMPLATE.md](TEMPLATE.md) when authoring requirements or architecture.
The [traceability map](../../formal/agent-review/traceability-map.json) registers
verification scope; an indexed requirement or linked proof is not a passing result.

## Core requirements

| ID | Required behavior |
|---|---|
| `E-PRED-ATTITUDE` | [Attitude prediction](requirements/core/attitude-prediction.md) |
| `E-PRED` | [Nominal prediction](requirements/core/prediction.md) |
| `E-PRED-INS` | [INS translation prediction](requirements/core/translation-prediction.md) |

## Runtime requirements

| ID | Required behavior |
|---|---|
| `R-RING-BUFFER` | [Fixed-capacity timestamped history](requirements/runtime/ring-buffer.md) |

## Additional contracts

The [requirement catalogue](verification-requirements.md) and its subject links
own requirements not defined in the entries above. A forwarding entry contains
no second definition.
