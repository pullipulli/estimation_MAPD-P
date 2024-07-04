# Idee

- [Congestion Bands](https://ora.ox.ac.uk/objects/uuid:70cee066-55ad-4ecf-a109-ca08a83b53ca/files/s7w62f842g)
  Si Potrebbe pensare di estendere il concetto di congestion band da 'archi' a 'path (insieme di archi)'
- [Traffic Load In Lane](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6958097)

  $TL_{Ei}$ -> Traffic Load in Lane i

  $L_{Ei}$ -> Length of Lane i

  $O_{Ei}$ -> Occupancy in Lane i

  $K_{Ei}$ -> Number of veichles in Lane i

  $L_v$ -> Average Length of the veichles

  $L_{gap}$ -> Average Length of the gap between veichles

  $$
  TL_{Ei} = O_{Ei} = K_{Ei}\frac{L_v + L_{gap}}{L_{Ei}}100\%
  $$

  Tutto questo si potrebbe 'tradurre' nel nostro caso come:

  $T_{Pi}$ -> Traffico nel path considerato
  $L_{Pi}$ -> Lunghezza del path considerato
  $K_{Pi}$ -> Numero di agenti nel path considerato
  $L_a$ -> Lunghezza media degli agenti (1)
  $L_{gap}$ -> Lunghezza media del gap tra agenti

  $$
  T_{Pi} = O_{Pi} = K_{Pi}\frac{1 + L_{gap}}{L_{pi}}100\%
  $$
