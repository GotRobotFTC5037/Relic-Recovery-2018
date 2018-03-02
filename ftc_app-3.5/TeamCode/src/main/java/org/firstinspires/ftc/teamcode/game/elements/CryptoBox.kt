package org.firstinspires.ftc.teamcode.game.elements

import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabber

/**
 * Used to keep track of the glyphs that are in a crypto box, primarily in autonomous.
 */
class CryptoBox {

    private val columns = listOf(Column(), Column(), Column())

    val isFull: Boolean
        get() = columns.all { it.isFull }

    fun addGlyphToColumn(glyph: Glyph, column: ColumnPosition) {
        columns[column.ordinal].addGlyph(glyph)
    }

    fun positionForGlyphs(glyphs: CodaGlyphGrabber.GrabbedGlyphs) =
            // What the heck have I created!? Well, if it works, it works. I'm just going to say
            // sorry right now to who ever is trying to understand what is going on here.
        columns.withIndex().firstOrNull { it.value.isFull.not() }?.let {
            Position(ColumnPosition.values()[it.index], it.value.firstOpenRow!!)
        }

    enum class ColumnPosition {
        LEFT, CENTER, RIGHT
    }

    enum class RowPosition {
        FIRST, SECOND, THIRD, FOURTH
    }

    data class Position(
        val column: ColumnPosition,
        val row: RowPosition
    )

    class Column {

        private val glyphs = mutableListOf<Glyph?>(null, null, null, null)

        val isFull: Boolean
            get() = glyphs.all { (it == null).not() }

        val firstOpenRow: RowPosition?
            get() = glyphs.withIndex().firstOrNull { it.value == null }?.let {
                RowPosition.values()[it.index]
            }

        fun addGlyph(glyph: Glyph) {
            firstOpenRow?.let { glyphs[it.ordinal] = glyph }
        }

    }

}
