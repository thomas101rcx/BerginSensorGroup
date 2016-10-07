######################################################################
# Visualization App
#
# Visualizes meaurements of sensors.
#
#
# Joshua Drawbaugh
# September, 2016
#
######################################################################


#################### Initialization ####################

# Libraries and functions to be used
library(shiny)
library(RColorBrewer)
source("Caller.R")
source("Functions/Default.R")


Folders = list.files("Data/Raw Data")

#################### User Interface ####################
ui <- fluidPage(
  titlePanel("Visualization"),
  
  
  fluidRow(
    column(3,
           selectInput("folder", label = "Choose Folder",
                              choices=Folders
                              )
           )

  ),
  
  fluidRow(
    column(3,
           checkboxGroupInput("sensor", label = "Choose Sensor", 
                              choices="Folder chosen is empty")
           )
  ),
  
  fluidRow(
    column(3,
           checkboxGroupInput("Pollutants", label = "Pollutants to plot",
                              c("PM2.5", "O3", "CO2", "NO2", "NO"), 
                              selected = "PM2.5"))
  ),
  
#  fluidRow(
    #column(3,
    
        #       sliderInput("Depended", label = "Dependent Range",
   #                           c("PM2.5", "O3", "CO2", "NO2", "NO"), 
  #                            selected = "PM2.5"))
 # ),
  #fluidRow(
   # column(3,
  #         numericInput(inputId = "ymax", "Vertical maximum", 1)),
    
  #  column(3,
  #         numericInput(inputId = "ymin", "Vertical minimum", 1))
  #)
  

  mainPanel(
    plotOutput("plot",click="plot_click",height = "700px")
  )  
  
)

#################### Server ####################
server <- function(input, output, session) {

  # Make checkbox of sensors  
  observe({
    # Chosen Folder
    x <- input$folder
    files <- list.files(paste("Data/Formatted Data/", x, sep=""))
    # Clean and format files in chosen folder
    CALLER(x)
    # Update checkboxes with sensor names
    updateCheckboxGroupInput(session, "sensor", choices = files)
    })

  # Make checkbox of plotting conditions
  
  # Plot
}


shinyApp(ui, server)
