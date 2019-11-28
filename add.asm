.ORIG x3000
      LEA R0, i
      LDW R1, R0, #0
      LDW R2, R1, #0
      ADD R2, R2, #1
      STW R2, R1, #0

      AND R6, R6, #0
      ADD R6, R6, #15
      ADD R6, R6, #4

      AND R3, R3, #0
      LEA R0, s
      LDW R1, R0, #0

loo   LDB R2, R1, #0
      ADD R3, R3, R2
      ADD R1, R1, #1
      ADD R6, R6, #-1
      BRZP loo

      LEA R0, e
      LDW R1, R0, #0
      STW R3, R1, #0
      HALT
e     .FILL x0000
s     .FILL xC000
i     .FILL x4000
.END
