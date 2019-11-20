.ORIG x1200
      ADD R6, R6 #-2          ;push
      STW R1, R6, #0
      ADD R6, R6, #-2
      STW R2, R6, #0

      LEA R1, loc             ;load R1 with address of x4000
      LDW R1, R1, #0          ;load R1 with x4000
      LDW R2, R1, #0          ;load R2 with contents of x4000
      ADD R2, R2 #1           ;increment contents by 1
      STW R2, R1, #0          ;store R2 at x4000

      LDW R2, R6, #0
      ADD R6, R6, #2
      LDW R1, R6, #0
      ADD R6, R6, #2         ;pop
      RTI
loc   .FILL x4000
sav   .FILL x4050
.END
