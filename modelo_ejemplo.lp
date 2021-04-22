\ Model modelo_ejemplo
\ LP format - for model browsing. Use MPS format to capture full model detail.
Maximize
  0 xe + 0 xi + 0 xe + 0 xi + 0 xe + 0 xi + 0 xe + 0 xi + 0 xe + 0 xi + 0 xe
   + 0 xi + 0 xe + 2 xi + 3 Constant
Subject To
 R0: xe + 2 xi <= 6
 R1: 2 xe + xi <= 8
 R2: - xe + xi <= 1
 R3: xi <= 2
 R4: 2 xi <= 5
 R5: xi <= 6
 R6: xi <= 2
 R7: xi <= 2
Bounds
 xe >= 1
 Constant = 1
End
