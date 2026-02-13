#import "@preview/tablem:0.3.0": tablem

#let tbl(body) = tablem(body)
#let sign(body) = {
  set text(font: "Allura")
  set image(height: 2em)
  show image: it => {
    box(it)
  }
  
  body
}

#let subtitle(body) = {
  show: block.with(width: 100%)
  set align(center)
  set text(weight: "bold", size: 1.25em)
  body
}

#let main-title(main, sub) = block(width: 100%, {
  set align(center)
  set text(weight: "bold")

  {
    set text(size: 1.5em)
    main
  }
  linebreak()
  {
    set text(size: 1em)
    sub
  }
})

#let signature-list(..sig-pairs) = {
  let colon = sig-pairs.pos().map(it => {
    (it.first() + ":", it.at(1))
  })
  let sigs = colon.flatten()

  set image(height: 2em)
  show image: box.with(stroke: (bottom: 1pt), inset: (top: 0em, rest: 0.25em))
  
  grid(
    align: horizon,
    columns: 2,
    gutter: 0.5em,
    rows: sigs.len(),
    ..sigs
  )
}

#let general(body) = {
  set text(size: 12pt, font: "Times New Roman")
  set title[ASTRO]

  set outline(title: none)
  set bibliography(title: none)

  set heading(numbering: "1.1)")

  show table: set align(center)
  set table(align: start)
  
  set par(first-line-indent: 0em, leading: 1em, spacing: 1.5em)
  set figure(placement: top)
  set figure.caption(separator: ". ")
  show figure.where(kind: table): set figure.caption(position: top)
  set footnote(numbering: "1")
  body
}