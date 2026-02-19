# Type of element: 0 = STAGE, 1 = REST
uint8 STAGE = 0
uint8 REST = 1

uint8 type

# Only populated if type == STAGE
ProtocolStageInfo stage

# Only populated if type == REST
ProtocolRestInfo rest
