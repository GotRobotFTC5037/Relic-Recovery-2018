package org.firstinspires.ftc.teamcode.game.elements

/**
 * Used to keep track of the glyphs that are in a crypto box, primarily in autonomous.
 */
class CryptoBox {

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
            .sortedBy { it.value.numberOfGlyphs }
            .firstOrNull { it.value.numberOfGlyphs % 2 != 0 && it.value.numberOfGlyphs < 2 }
            ?.let {
                Position(
                    ColumnPosition.values()[it.index],
                    it.value.firstOpenRow!!
                )
            }

    /** Adds a [glyph] to the selected [column]. */
    fun addGlyphToColumn(glyph: Glyph, column: ColumnPosition) =
        columns[column.ordinal].addGlyph(glyph)

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

}

