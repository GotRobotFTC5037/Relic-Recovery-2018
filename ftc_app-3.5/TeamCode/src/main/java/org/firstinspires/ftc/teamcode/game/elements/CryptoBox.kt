package org.firstinspires.ftc.teamcode.game.elements

/**
 * Used to keep track of the glyphs that are in a crypto box, primarily in autonomous.
 */
class CryptoBox constructor() {

    private constructor(glyphs: List<List<Glyph>>) : this() {
        glyphs.mapIndexed { columnIndex, column ->
            column.mapIndexed { rowIndex, glyph ->
                columns[columnIndex].glyphs[rowIndex] = glyph
            }
        }
    }

    private val columns = listOf(Column(), Column(), Column())

    /** A collection of glyphs stacked into a column. */
    private class Column {

        val glyphs = mutableListOf<Glyph?>(null, null, null, null)

        val isFull: Boolean
            get() = glyphs.all { (it == null).not() }

        val numberOfGlyphs: Int
            get() = glyphs.count { (it == null).not() }

        val firstOpenRow: RowPosition?
            get() = glyphs
                .withIndex()
                .firstOrNull { it.value == null }
                ?.let { RowPosition.values()[it.index] }

        fun addGlyph(glyph: Glyph) = firstOpenRow?.let { glyphs[it.ordinal] = glyph }

    }

    /** The current fill state of the crypto box. */
    val isFull: Boolean
        get() = columns.all { it.isFull }

    /** Returns the best place to put the next glyph. */
    fun positionForNextGlyph() =
        columns
            .withIndex()
            .filter { it.value.isFull.not() }
            .sortedBy { it.value.numberOfGlyphs }
            .firstOrNull()
            ?.let { Position(ColumnPosition.values()[it.index], it.value.firstOpenRow!!) }

    /** Adds a [glyph] to the selected [column]. */
    fun addGlyphToColumn(glyph: Glyph, column: ColumnPosition) =
        columns[column.ordinal].addGlyph(glyph)

    private fun inverted(): CryptoBox {
        return CryptoBox().also {
            for ((i, column) in columns.withIndex()) {
                column.glyphs.withIndex()
                    .forEach { (index, glyph) ->
                        it.columns[i].glyphs[index] = glyph?.let {
                            Glyph(
                                when (it.color) {
                                    Glyph.Color.GRAY -> Glyph.Color.BROWN
                                    Glyph.Color.BROWN -> Glyph.Color.GRAY
                                    Glyph.Color.UNKNOWN -> Glyph.Color.UNKNOWN
                                }
                            )
                        }
                    }
            }
        }
    }

    /** A column position within the crypto box. */
    enum class ColumnPosition {

        /** The left column. */
        LEFT,

        /** The center column. */
        CENTER,

        /** The right column. */
        RIGHT

    }

    /** A row position within a crypto box. */
    enum class RowPosition {

        /** The first row. */
        FIRST,

        /** The second row. */
        SECOND,

        /** The third row. */
        THIRD,

        /** The fourth row. */
        FOURTH

    }

    /** A position within a crypto box. */
    data class Position(

        /** The column of the crypto box. */
        val column: ColumnPosition,

        /** The row of the crypto box. */
        val row: RowPosition

    )

    /** A crypto box filled into a pattern that makes a cipher. */
    enum class Cipher {

        FROG {
            override val template: CryptoBox
                get() = CryptoBox(
                    listOf(
                        listOf(
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY)
                        ),
                        listOf(
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN)
                        ),
                        listOf(
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY)
                        )
                    )
                )
        },

        FROG_INVERTED {
            override val template: CryptoBox
                get() = FROG.template.inverted()
        },

        BIRD {
            override val template: CryptoBox
                get() = CryptoBox(
                    listOf(
                        listOf(
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY)
                        ),
                        listOf(
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN)
                        ),
                        listOf(
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY)
                        )
                    )
                )
        },

        BIRD_INVERTED {
            override val template: CryptoBox
                get() = BIRD.template.inverted()
        },

        SNAKE {
            override val template: CryptoBox
                get() = CryptoBox(
                    listOf(
                        listOf(
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.BROWN)
                        ),
                        listOf(
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY)
                        ),
                        listOf(
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.BROWN),
                            Glyph(Glyph.Color.GRAY),
                            Glyph(Glyph.Color.GRAY)
                        )
                    )
                )
        },

        SNAKE_INVERTED {
            override val template: CryptoBox
                get() = SNAKE.template.inverted()
        };

        /** A crypto box filled with a cipher */
        abstract val template: CryptoBox
    }

}

