USE [DBWaterQuality]
GO

/****** Object:  View [dbo].[INWSStaticDataGroupedByTimestamp]    Script Date: 10/27/2020 2:13:56 PM ******/
SET ANSI_NULLS ON
GO

SET QUOTED_IDENTIFIER ON
GO

IF EXISTS (SELECT TABLE_NAME FROM INFORMATION_SCHEMA.VIEWS
        WHERE TABLE_NAME = 'INWSStaticDataGroupedByTimestamp')
    DROP VIEW INWSStaticDataGroupedByTimestamp
GO

CREATE VIEW [dbo].[INWSStaticDataGroupedByTimestamp]
AS
(SELECT Timestamp, NodeId, 
	MAX(CASE WHEN Parameter = 'Temperature' THEN Value END) AS Temperature, 
    MAX(CASE WHEN Parameter = 'Conductivity' THEN Value END) AS Conductivity, 
	MAX(CASE WHEN Parameter = 'pH' THEN Value END) AS pH, 
	MAX(CASE WHEN Parameter = 'DissolvedOxygen' THEN Value END) AS DissolvedOxygen
FROM     dbo.INWSStaticDataParameterWise
GROUP BY Timestamp, NodeId
ORDER BY NodeId, Timestamp)
GO