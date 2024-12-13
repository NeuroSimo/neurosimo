import React, { useState } from 'react'

import { useTable, Row, useExpanded } from 'react-table'
import styled from 'styled-components'
import { ControlledMenu, MenuItem, useMenuState } from '@szhsin/react-menu'
import { menuSelector, menuItemSelector, menuDividerSelector } from '@szhsin/react-menu/style-utils'

import '@szhsin/react-menu/dist/index.css'
import NotEditableCell from './TableElements/Cells/NotEditableCell'
import { DndProvider } from 'react-dnd'
import { HTML5Backend } from 'react-dnd-html5-backend'
import { useAppSelector } from '../providers/reduxHooks'

const defaultColumn = {
  Cell: NotEditableCell,
}

interface TableProps {
  columns: any[]
  data: any[]
  createMenu: any
  SelectableRow: any
}

export const GenericTable = ({ columns, data, createMenu, SelectableRow }: TableProps) => {
  const { expandedSequences } = useAppSelector((state) => state.sequences)
  const {
    getTableProps,
    getTableBodyProps,
    headerGroups,
    prepareRow,
    rows,
    state: { expanded },
  } = useTable(
    {
      columns,
      data,
      defaultColumn,
      autoResetExpanded: false,
      initialState: { expanded: expandedSequences },
    },
    useExpanded,
  )

  const [menuProps, toggleMenu] = useMenuState()
  const [anchorPoint, setAnchorPoint] = useState({ x: 0, y: 0 })

  const onContextMenu = (event: any) => {
    event.preventDefault()
    setAnchorPoint({ x: event.clientX, y: event.clientY })
    toggleMenu(true)
  }

  return (
    <MenuContainer>
      <TargetsContainer onContextMenu={onContextMenu}>
        <DndProvider backend={HTML5Backend}>
          <TargetsTable {...getTableProps()} id='targets-table'>
            <Thead>
              {headerGroups.map((headerGroup) => (
                <HeaderTableRow {...headerGroup.getHeaderGroupProps()} key={headerGroup.getHeaderGroupProps().key}>
                  <MoveTh></MoveTh>
                  {headerGroup.headers.map((column) => (
                    <Th
                      {...column.getHeaderProps({
                        style: {
                          width: column.width,
                          minWidth: column.minWidth,
                        },
                      })}
                      key={column.getHeaderProps().key}
                    >
                      {column.render('Header')}
                    </Th>
                  ))}
                </HeaderTableRow>
              ))}
            </Thead>

            <Tbody {...getTableBodyProps()}>
              {rows.map((row: Row) => {
                prepareRow(row)
                const isTarget = row.depth > 0 || (row.depth === 0 && row.subRows.length === 0)
                return (
                  <SelectableRow
                    {...row.getRowProps()}
                    key={row.getRowProps().key}
                    index={row.index}
                    isTarget={isTarget}
                    rowId={row.id}
                  >
                    {row.cells.map((cell) => {
                      return (
                        <Td {...cell.getCellProps()} key={cell.getCellProps().key}>
                          {cell.render('Cell')}
                        </Td>
                      )
                    })}
                  </SelectableRow>
                )
              })}
            </Tbody>
          </TargetsTable>
        </DndProvider>
      </TargetsContainer>
      <StyledMenu {...menuProps} anchorPoint={anchorPoint} onClose={() => toggleMenu(false)} overflow='visible'>
        {createMenu()}
      </StyledMenu>
    </MenuContainer>
  )
}

const MenuContainer = styled.div`
  all: unset;
  overflow: visible;
`

const StyledMenu = styled(ControlledMenu)`
  ${menuSelector.name} {
    border: 1px solid ${(p) => p.theme.colors.gray};
  }

  z-index: 1;
`

const Th = styled.th`
  padding: 0.25rem 0.5rem;
  text-align: left;
  border-top: none !important;
  border-bottom: none !important;

  :last-of-type {
    box-shadow: inset 0 1px 0 ${(p) => p.theme.colors.gray}, inset 0 -1px 0 ${(p) => p.theme.colors.gray};
  }

  :not(:last-of-type) {
    box-shadow: inset 0 1px 0 ${(p) => p.theme.colors.gray}, inset 0 -1px 0 ${(p) => p.theme.colors.gray},
      inset -1px 0 0 ${(p) => p.theme.colors.gray};
  }
`

const MoveTh = styled(Th)`
  width: 10px;
`

const Td = styled.td`
  padding: 0.25rem 0.5rem;
  border: #e0e0e0;
`

const Thead = styled.thead`
  position: sticky;
  top: 0;
  margin: 0 0 0 0;
  width: 100%;
  z-index: 1;
`
const Tbody = styled.tbody``

const HeaderTableRow = styled.tr`
  :nth-of-type(even) {
    background-color: #f3f3f3;
  }

  :nth-of-type(odd) {
    background-color: #ffffff;
  }
`

const TargetsContainer = styled.div`
  overflow-y: auto;
  overflow-x: hidden;
  width: fit-content;
  max-height: 600px;
`

const TargetsTable = styled.table`
  border-collapse: collapse;
  box-shadow: 0 0 20px rgba(0, 0, 0, 0.15);
  table-layout: fixed;
  width: 100%;
`
