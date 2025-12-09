import theme from '../../src/styles/theme'
import { clearRosState } from '../../src/services/ros'

const notSelectedColor = theme.colors.white
const selectedColor = theme.colors.lightgray

const defaultWaitDuration = 500

const testUrl = `http://${Cypress.env('FRONT_URL') ?? 'localhost:3000'}`
describe('Target table', () => {
  before(() => {
    cy.visit(testUrl + '/targets')
  })
  beforeEach(() => {
    clearRosState()
    cy.wait(defaultWaitDuration)
  })

  it('can be opened', () => {
    cy.contains('NeuroSimo')
  })
/* XXX: The following tests have bitrotten: #add-target-button element cannot be found. Before fixing and re-enabling,
     ensure that the targeting table is actually something we want to do. For now, comment them out so that they won't be
     forgotten.

  it('allows adding new targets', () => {
    cy.get('#add-target-button').click()
    cy.get('#add-target-button').click()
    cy.wait(defaultWaitDuration)
    cy.get('#targets-table')
      .find('tr')
      .then((rows) => {
        const originalLength = rows.toArray().length

        cy.get('#add-target-button').click()
        cy.wait(defaultWaitDuration)

        cy.get('#targets-table')
          .find('tr')
          .then((newRows) => {
            expect(newRows.toArray().length).to.equal(originalLength + 1)
          })
      })
  })

  it('allows editing targets', () => {
    cy.get('#add-target-button').click()
    cy.wait(defaultWaitDuration)

    cy.get('#cell-container-0-name').find('.cell-value-container').first().dblclick()
    cy.get('#cell-container-0-name').find('input').clear().type('first target')
    cy.get('#cell-container-0-name').find('input').type('{enter}')
  })

  it('allows adding sequences and editing them', () => {
    cy.get('#add-target-button').click()
    cy.wait(defaultWaitDuration)
    cy.get('#add-target-button').click()
    cy.wait(defaultWaitDuration)

    cy.get('#target-0').click()
    cy.get('#target-1').click()

    cy.get('#targets-table').rightclick()

    cy.get('#create-new-sequence').click()
    cy.get('#sequences-view-button').click()

    //sequence-0 + header row
    cy.get('#targets-table').find('tr').its('length').should('equal', 2)

    const newName = 'a sequence'

    cy.get('#cell-container-0-seqName').find('.cell-value-container').first().dblclick()
    cy.get('#cell-container-0-seqName').find('input').clear().type(newName)
    cy.get('#cell-container-0-seqName').find('input').type('{enter}')

    cy.reload()
    cy.get('#sequences-view-button').click()
    cy.get('#targets-table').find('tr').its('length').should('equal', 2)

    cy.contains(newName)

    cy.get('#targets-view-button').click()
    cy.get('#targets-table').rightclick()

    cy.contains('Add to sequence').click()
    cy.get('#add-to-sequence-0').click()

    cy.get('#sequences-view-button').click()
    cy.get('#cell-container-0-seqName').find('.cell-value-container').first().find('button').click()
    cy.get('#targets-table').find('tr').its('length').should('equal', 6)
  })
*/
  after(() => {
    clearRosState()
  })
})
