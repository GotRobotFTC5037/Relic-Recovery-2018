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

    fun positionForGlyphs(glyphs: CodaGlyphGrabber.GrabbedGlyphs): Position? {
        val availableColumns = columns.withIndex().filter { it.value.isFull.not() }
        return availableColumns.firstOrNull()?.let {
            Position(ColumnPosition.values()[it.index], it.value.firstOpenRow!!)
        }
    }

    enum class ColumnPosition {
        LEFT, CENTER, RIGHT
    }

    enum class RowPosition {
        FIRST, SECOND, THIRD, FOURTH
    }

    data class Position (
        val column: ColumnPosition,
        val row: RowPosition
    )

    class Column {

        private val glyphs = mutableListOf<Glyph?>(null, null, null, null)

        val isFull: Boolean
            get() = glyphs.all { (it == null).not() }

        val firstOpenRow: RowPosition?
            get() {
                for ((row, glyph) in glyphs.withIndex()) {
                    return glyph?.let { RowPosition.values()[row] }
                }
                return null
            }

        fun addGlyph(glyph: Glyph) {
            val row = firstOpenRow
            row?.let {
                glyphs[it.ordinal] = glyph
            }
        }

    }

}
