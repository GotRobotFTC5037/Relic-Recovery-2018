package org.firstinspires.ftc.teamcode.game.elements

import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabber

/**
 * Used to keep track of the glyphs that are in a crypto box, primarily in autonomous.
 */
class CryptoBox {

    private val columns = listOf(Column(), Column(), Column())

    /** A collection of glyphs stacked into a column. */
    class Column {

        private val glyphs = mutableListOf<Glyph?>(null, null, null, null)

        /** The current fill state of the column. */
        val isFull: Boolean
            get() = glyphs.all { (it == null).not() }

        /** The first open row from the bottom of the column. */
        val firstOpenRow: RowPosition?
            get() = glyphs
                .withIndex()
                .firstOrNull { it.value == null }
                ?.let { RowPosition.values()[it.index] }

        /** Adds the provided [glyph] into the column */
        fun addGlyph(glyph: Glyph) {
            firstOpenRow?.let { glyphs[it.ordinal] = glyph }
        }

    }

    /** The current fill state of the crypto box. */
    val isFull: Boolean
        get() = columns.all { it.isFull }

    /** Adds a [glyph] to the selected [column]. */
    fun addGlyphToColumn(glyph: Glyph, column: ColumnPosition) {
        columns[column.ordinal].addGlyph(glyph)
    }

    /** Returns the best place to put the [glyphs]. */
    fun positionForGlyphs(glyphs: CodaGlyphGrabber.GrabbedGlyphs) =
        columns
            .sortedBy { it.firstOpenRow?.ordinal }
            .withIndex()
            .firstOrNull { it.value.isFull.not() }
            ?.let {
                Position(
                    column = ColumnPosition.values()[it.index],
                    row = it.value.firstOpenRow!!
                )
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
        val column: ColumnPosition,
        val row: RowPosition
    )

}
